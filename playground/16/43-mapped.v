// Benchmark "adder" written by ABC on Wed Jul 17 20:33:25 2024

module adder ( 
    \a[0] , \a[10] , \a[11] , \a[12] , \a[13] , \a[14] , \a[15] , \a[16] ,
    \a[17] , \a[18] , \a[19] , \a[1] , \a[20] , \a[21] , \a[22] , \a[23] ,
    \a[24] , \a[25] , \a[26] , \a[27] , \a[28] , \a[29] , \a[2] , \a[30] ,
    \a[31] , \a[3] , \a[4] , \a[5] , \a[6] , \a[7] , \a[8] , \a[9] ,
    \b[0] , \b[10] , \b[11] , \b[12] , \b[13] , \b[14] , \b[15] , \b[16] ,
    \b[17] , \b[18] , \b[19] , \b[1] , \b[20] , \b[21] , \b[22] , \b[23] ,
    \b[24] , \b[25] , \b[26] , \b[27] , \b[28] , \b[29] , \b[2] , \b[30] ,
    \b[3] , \b[4] , \b[5] , \b[6] , \b[7] , \b[8] , \b[9] ,
    \s[0] , \s[10] , \s[11] , \s[12] , \s[13] , \s[14] , \s[15] , \s[16] ,
    \s[17] , \s[18] , \s[19] , \s[1] , \s[20] , \s[21] , \s[22] , \s[23] ,
    \s[24] , \s[25] , \s[26] , \s[27] , \s[28] , \s[29] , \s[2] , \s[30] ,
    \s[31] , \s[3] , \s[4] , \s[5] , \s[6] , \s[7] , \s[8] , \s[9]   );
  input  \a[0] , \a[10] , \a[11] , \a[12] , \a[13] , \a[14] , \a[15] ,
    \a[16] , \a[17] , \a[18] , \a[19] , \a[1] , \a[20] , \a[21] , \a[22] ,
    \a[23] , \a[24] , \a[25] , \a[26] , \a[27] , \a[28] , \a[29] , \a[2] ,
    \a[30] , \a[31] , \a[3] , \a[4] , \a[5] , \a[6] , \a[7] , \a[8] ,
    \a[9] , \b[0] , \b[10] , \b[11] , \b[12] , \b[13] , \b[14] , \b[15] ,
    \b[16] , \b[17] , \b[18] , \b[19] , \b[1] , \b[20] , \b[21] , \b[22] ,
    \b[23] , \b[24] , \b[25] , \b[26] , \b[27] , \b[28] , \b[29] , \b[2] ,
    \b[30] , \b[3] , \b[4] , \b[5] , \b[6] , \b[7] , \b[8] , \b[9] ;
  output \s[0] , \s[10] , \s[11] , \s[12] , \s[13] , \s[14] , \s[15] ,
    \s[16] , \s[17] , \s[18] , \s[19] , \s[1] , \s[20] , \s[21] , \s[22] ,
    \s[23] , \s[24] , \s[25] , \s[26] , \s[27] , \s[28] , \s[29] , \s[2] ,
    \s[30] , \s[31] , \s[3] , \s[4] , \s[5] , \s[6] , \s[7] , \s[8] ,
    \s[9] ;
  wire new_n97, new_n98, new_n99, new_n100, new_n101, new_n102, new_n103,
    new_n104, new_n105, new_n106, new_n107, new_n108, new_n109, new_n110,
    new_n111, new_n112, new_n113, new_n114, new_n115, new_n116, new_n117,
    new_n118, new_n119, new_n120, new_n121, new_n122, new_n123, new_n124,
    new_n125, new_n126, new_n127, new_n128, new_n129, new_n130, new_n132,
    new_n133, new_n134, new_n136, new_n137, new_n138, new_n139, new_n140,
    new_n141, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n162, new_n164,
    new_n165, new_n166, new_n167, new_n168, new_n169, new_n170, new_n171,
    new_n172, new_n173, new_n174, new_n175, new_n176, new_n177, new_n178,
    new_n179, new_n180, new_n181, new_n182, new_n183, new_n184, new_n185,
    new_n187, new_n188, new_n189, new_n190, new_n191, new_n192, new_n193,
    new_n195, new_n196, new_n197, new_n198, new_n199, new_n200, new_n201,
    new_n202, new_n203, new_n204, new_n205, new_n206, new_n207, new_n208,
    new_n209, new_n210, new_n211, new_n212, new_n213, new_n214, new_n216,
    new_n217, new_n218, new_n219, new_n221, new_n222, new_n223, new_n224,
    new_n225, new_n226, new_n227, new_n228, new_n229, new_n230, new_n233,
    new_n234, new_n235, new_n236, new_n238, new_n239, new_n240, new_n241,
    new_n242, new_n243, new_n244, new_n245, new_n246, new_n247, new_n248,
    new_n249, new_n251, new_n252, new_n253, new_n254, new_n255, new_n256,
    new_n258, new_n259, new_n260, new_n261, new_n262, new_n263, new_n264,
    new_n265, new_n266, new_n268, new_n269, new_n270, new_n271, new_n272,
    new_n273, new_n274, new_n275, new_n277, new_n278, new_n279, new_n280,
    new_n281, new_n282, new_n284, new_n285, new_n286, new_n287, new_n288,
    new_n289, new_n290, new_n291, new_n293, new_n294, new_n295, new_n296,
    new_n297, new_n298, new_n299, new_n300, new_n301, new_n302, new_n304,
    new_n305, new_n306, new_n307, new_n308, new_n309, new_n310, new_n312,
    new_n313, new_n314, new_n315, new_n316, new_n317, new_n318, new_n319,
    new_n322, new_n323, new_n324, new_n325, new_n326, new_n327, new_n328,
    new_n330, new_n331, new_n332, new_n333, new_n334, new_n335, new_n336,
    new_n337, new_n340, new_n341, new_n342, new_n345, new_n346, new_n347,
    new_n349, new_n351, new_n352, new_n353;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  xnrc02aa1n12x5               g001(.a(\b[9] ), .b(\a[10] ), .out0(new_n97));
  inv000aa1d42x5               g002(.a(new_n97), .o1(new_n98));
  inv000aa1d42x5               g003(.a(\a[9] ), .o1(new_n99));
  nanb02aa1d24x5               g004(.a(\b[8] ), .b(new_n99), .out0(new_n100));
  nor042aa1n06x5               g005(.a(\b[2] ), .b(\a[3] ), .o1(new_n101));
  nand42aa1d28x5               g006(.a(\b[2] ), .b(\a[3] ), .o1(new_n102));
  nanb02aa1n12x5               g007(.a(new_n101), .b(new_n102), .out0(new_n103));
  nand42aa1n04x5               g008(.a(\b[1] ), .b(\a[2] ), .o1(new_n104));
  nor002aa1n02x5               g009(.a(\b[1] ), .b(\a[2] ), .o1(new_n105));
  nand22aa1n03x5               g010(.a(\b[0] ), .b(\a[1] ), .o1(new_n106));
  oai012aa1n02x7               g011(.a(new_n104), .b(new_n105), .c(new_n106), .o1(new_n107));
  oab012aa1n04x5               g012(.a(new_n101), .b(\a[4] ), .c(\b[3] ), .out0(new_n108));
  oai012aa1n04x7               g013(.a(new_n108), .b(new_n107), .c(new_n103), .o1(new_n109));
  nor002aa1d32x5               g014(.a(\b[4] ), .b(\a[5] ), .o1(new_n110));
  inv000aa1d42x5               g015(.a(new_n110), .o1(new_n111));
  aoi022aa1n02x5               g016(.a(\b[5] ), .b(\a[6] ), .c(\a[5] ), .d(\b[4] ), .o1(new_n112));
  nor022aa1n06x5               g017(.a(\b[7] ), .b(\a[8] ), .o1(new_n113));
  aoi012aa1n02x7               g018(.a(new_n113), .b(\a[4] ), .c(\b[3] ), .o1(new_n114));
  aoi022aa1n02x5               g019(.a(\b[7] ), .b(\a[8] ), .c(\a[7] ), .d(\b[6] ), .o1(new_n115));
  norp02aa1n04x5               g020(.a(\b[6] ), .b(\a[7] ), .o1(new_n116));
  nor002aa1n06x5               g021(.a(\b[5] ), .b(\a[6] ), .o1(new_n117));
  nona22aa1n02x4               g022(.a(new_n115), .b(new_n116), .c(new_n117), .out0(new_n118));
  nano32aa1n03x7               g023(.a(new_n118), .b(new_n114), .c(new_n112), .d(new_n111), .out0(new_n119));
  oai022aa1d24x5               g024(.a(\a[7] ), .b(\b[6] ), .c(\b[7] ), .d(\a[8] ), .o1(new_n120));
  nanp02aa1n12x5               g025(.a(\b[7] ), .b(\a[8] ), .o1(new_n121));
  oai012aa1n02x5               g026(.a(new_n121), .b(new_n116), .c(new_n113), .o1(new_n122));
  nanp02aa1n04x5               g027(.a(\b[5] ), .b(\a[6] ), .o1(new_n123));
  nanp02aa1n06x5               g028(.a(\b[6] ), .b(\a[7] ), .o1(new_n124));
  nand03aa1n02x5               g029(.a(new_n121), .b(new_n123), .c(new_n124), .o1(new_n125));
  nor022aa1n04x5               g030(.a(new_n117), .b(new_n110), .o1(new_n126));
  oai013aa1n03x5               g031(.a(new_n122), .b(new_n125), .c(new_n126), .d(new_n120), .o1(new_n127));
  xnrc02aa1n12x5               g032(.a(\b[8] ), .b(\a[9] ), .out0(new_n128));
  inv000aa1d42x5               g033(.a(new_n128), .o1(new_n129));
  aoai13aa1n03x5               g034(.a(new_n129), .b(new_n127), .c(new_n119), .d(new_n109), .o1(new_n130));
  xnbna2aa1n03x5               g035(.a(new_n98), .b(new_n130), .c(new_n100), .out0(\s[10] ));
  oaoi03aa1n12x5               g036(.a(\a[10] ), .b(\b[9] ), .c(new_n100), .o1(new_n132));
  inv000aa1d42x5               g037(.a(new_n132), .o1(new_n133));
  aoai13aa1n03x5               g038(.a(new_n133), .b(new_n97), .c(new_n130), .d(new_n100), .o1(new_n134));
  xorb03aa1n02x5               g039(.a(new_n134), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  nor002aa1d32x5               g040(.a(\b[11] ), .b(\a[12] ), .o1(new_n136));
  inv000aa1d42x5               g041(.a(new_n136), .o1(new_n137));
  nand02aa1d20x5               g042(.a(\b[11] ), .b(\a[12] ), .o1(new_n138));
  nor042aa1n06x5               g043(.a(\b[10] ), .b(\a[11] ), .o1(new_n139));
  nand02aa1n06x5               g044(.a(\b[10] ), .b(\a[11] ), .o1(new_n140));
  aoi012aa1n03x5               g045(.a(new_n139), .b(new_n134), .c(new_n140), .o1(new_n141));
  xnbna2aa1n03x5               g046(.a(new_n141), .b(new_n137), .c(new_n138), .out0(\s[12] ));
  nona23aa1n09x5               g047(.a(new_n138), .b(new_n140), .c(new_n139), .d(new_n136), .out0(new_n143));
  norp03aa1n06x5               g048(.a(new_n143), .b(new_n128), .c(new_n97), .o1(new_n144));
  aoai13aa1n03x5               g049(.a(new_n144), .b(new_n127), .c(new_n119), .d(new_n109), .o1(new_n145));
  nanb03aa1n12x5               g050(.a(new_n136), .b(new_n138), .c(new_n140), .out0(new_n146));
  nand42aa1n02x5               g051(.a(\b[9] ), .b(\a[10] ), .o1(new_n147));
  oai022aa1d24x5               g052(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n148));
  oai112aa1n06x5               g053(.a(new_n148), .b(new_n147), .c(\b[10] ), .d(\a[11] ), .o1(new_n149));
  aoi012aa1d18x5               g054(.a(new_n136), .b(new_n139), .c(new_n138), .o1(new_n150));
  oai012aa1d24x5               g055(.a(new_n150), .b(new_n149), .c(new_n146), .o1(new_n151));
  inv000aa1d42x5               g056(.a(new_n151), .o1(new_n152));
  nanp02aa1n02x5               g057(.a(new_n145), .b(new_n152), .o1(new_n153));
  nor022aa1n16x5               g058(.a(\b[12] ), .b(\a[13] ), .o1(new_n154));
  nand02aa1n06x5               g059(.a(\b[12] ), .b(\a[13] ), .o1(new_n155));
  norb02aa1n02x5               g060(.a(new_n155), .b(new_n154), .out0(new_n156));
  inv000aa1d42x5               g061(.a(new_n138), .o1(new_n157));
  oai012aa1n02x5               g062(.a(new_n140), .b(\b[11] ), .c(\a[12] ), .o1(new_n158));
  nona32aa1n06x5               g063(.a(new_n132), .b(new_n158), .c(new_n157), .d(new_n139), .out0(new_n159));
  nano22aa1n02x4               g064(.a(new_n156), .b(new_n159), .c(new_n150), .out0(new_n160));
  aoi022aa1n02x5               g065(.a(new_n153), .b(new_n156), .c(new_n145), .d(new_n160), .o1(\s[13] ));
  aoi012aa1n02x5               g066(.a(new_n154), .b(new_n153), .c(new_n155), .o1(new_n162));
  xnrb03aa1n02x5               g067(.a(new_n162), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  norb02aa1n02x7               g068(.a(new_n102), .b(new_n101), .out0(new_n164));
  oai112aa1n06x5               g069(.a(\a[1] ), .b(\b[0] ), .c(\b[1] ), .d(\a[2] ), .o1(new_n165));
  nanp03aa1n02x5               g070(.a(new_n164), .b(new_n104), .c(new_n165), .o1(new_n166));
  nand02aa1n03x5               g071(.a(\b[4] ), .b(\a[5] ), .o1(new_n167));
  nano22aa1n03x7               g072(.a(new_n110), .b(new_n123), .c(new_n167), .out0(new_n168));
  oai022aa1n02x5               g073(.a(\a[6] ), .b(\b[5] ), .c(\b[6] ), .d(\a[7] ), .o1(new_n169));
  nano22aa1n03x5               g074(.a(new_n169), .b(new_n121), .c(new_n124), .out0(new_n170));
  nand03aa1n02x5               g075(.a(new_n170), .b(new_n168), .c(new_n114), .o1(new_n171));
  and003aa1n02x5               g076(.a(new_n123), .b(new_n124), .c(new_n121), .o(new_n172));
  oab012aa1n02x5               g077(.a(new_n120), .b(new_n110), .c(new_n117), .out0(new_n173));
  aoi022aa1n02x7               g078(.a(new_n173), .b(new_n172), .c(new_n121), .d(new_n120), .o1(new_n174));
  aoai13aa1n03x5               g079(.a(new_n174), .b(new_n171), .c(new_n166), .d(new_n108), .o1(new_n175));
  norp02aa1n09x5               g080(.a(\b[13] ), .b(\a[14] ), .o1(new_n176));
  nanp02aa1n04x5               g081(.a(\b[13] ), .b(\a[14] ), .o1(new_n177));
  nano23aa1n03x5               g082(.a(new_n154), .b(new_n176), .c(new_n177), .d(new_n155), .out0(new_n178));
  aoai13aa1n02x5               g083(.a(new_n178), .b(new_n151), .c(new_n175), .d(new_n144), .o1(new_n179));
  oai012aa1n02x5               g084(.a(new_n177), .b(new_n176), .c(new_n154), .o1(new_n180));
  inv040aa1d32x5               g085(.a(\a[15] ), .o1(new_n181));
  inv000aa1d42x5               g086(.a(\b[14] ), .o1(new_n182));
  nand02aa1n02x5               g087(.a(new_n182), .b(new_n181), .o1(new_n183));
  nanp02aa1n02x5               g088(.a(\b[14] ), .b(\a[15] ), .o1(new_n184));
  nand42aa1n02x5               g089(.a(new_n183), .b(new_n184), .o1(new_n185));
  xobna2aa1n03x5               g090(.a(new_n185), .b(new_n179), .c(new_n180), .out0(\s[15] ));
  inv040aa1d28x5               g091(.a(\a[16] ), .o1(new_n187));
  inv000aa1d42x5               g092(.a(\b[15] ), .o1(new_n188));
  nand22aa1n06x5               g093(.a(new_n188), .b(new_n187), .o1(new_n189));
  nanp02aa1n09x5               g094(.a(\b[15] ), .b(\a[16] ), .o1(new_n190));
  nona23aa1n09x5               g095(.a(new_n177), .b(new_n155), .c(new_n154), .d(new_n176), .out0(new_n191));
  aoai13aa1n03x5               g096(.a(new_n180), .b(new_n191), .c(new_n145), .d(new_n152), .o1(new_n192));
  oaoi03aa1n03x5               g097(.a(new_n181), .b(new_n182), .c(new_n192), .o1(new_n193));
  xnbna2aa1n03x5               g098(.a(new_n193), .b(new_n189), .c(new_n190), .out0(\s[16] ));
  nano23aa1n02x4               g099(.a(new_n139), .b(new_n136), .c(new_n138), .d(new_n140), .out0(new_n195));
  nanp02aa1n02x5               g100(.a(new_n189), .b(new_n190), .o1(new_n196));
  nona22aa1n02x4               g101(.a(new_n178), .b(new_n185), .c(new_n196), .out0(new_n197));
  nano32aa1n03x7               g102(.a(new_n197), .b(new_n129), .c(new_n98), .d(new_n195), .out0(new_n198));
  nanp02aa1n02x5               g103(.a(new_n175), .b(new_n198), .o1(new_n199));
  aoi012aa1n12x5               g104(.a(new_n127), .b(new_n119), .c(new_n109), .o1(new_n200));
  nor043aa1n03x5               g105(.a(new_n191), .b(new_n185), .c(new_n196), .o1(new_n201));
  nand22aa1n03x5               g106(.a(new_n201), .b(new_n144), .o1(new_n202));
  nanp03aa1n02x5               g107(.a(new_n189), .b(new_n184), .c(new_n190), .o1(new_n203));
  oai112aa1n03x5               g108(.a(new_n177), .b(new_n183), .c(new_n176), .d(new_n154), .o1(new_n204));
  nor002aa1n02x5               g109(.a(\b[14] ), .b(\a[15] ), .o1(new_n205));
  aob012aa1d15x5               g110(.a(new_n189), .b(new_n205), .c(new_n190), .out0(new_n206));
  inv000aa1n02x5               g111(.a(new_n206), .o1(new_n207));
  oai012aa1n02x7               g112(.a(new_n207), .b(new_n204), .c(new_n203), .o1(new_n208));
  aoi012aa1n12x5               g113(.a(new_n208), .b(new_n201), .c(new_n151), .o1(new_n209));
  oai012aa1n18x5               g114(.a(new_n209), .b(new_n200), .c(new_n202), .o1(new_n210));
  xorc02aa1n02x5               g115(.a(\a[17] ), .b(\b[16] ), .out0(new_n211));
  norp02aa1n02x5               g116(.a(new_n204), .b(new_n203), .o1(new_n212));
  orn002aa1n02x5               g117(.a(new_n206), .b(new_n211), .o(new_n213));
  aoi112aa1n02x5               g118(.a(new_n212), .b(new_n213), .c(new_n201), .d(new_n151), .o1(new_n214));
  aoi022aa1n02x5               g119(.a(new_n210), .b(new_n211), .c(new_n199), .d(new_n214), .o1(\s[17] ));
  inv040aa1d32x5               g120(.a(\a[18] ), .o1(new_n216));
  inv040aa1d32x5               g121(.a(\a[17] ), .o1(new_n217));
  inv040aa1d28x5               g122(.a(\b[16] ), .o1(new_n218));
  oaoi03aa1n02x5               g123(.a(new_n217), .b(new_n218), .c(new_n210), .o1(new_n219));
  xorb03aa1n02x5               g124(.a(new_n219), .b(\b[17] ), .c(new_n216), .out0(\s[18] ));
  oab012aa1n02x4               g125(.a(new_n206), .b(new_n204), .c(new_n203), .out0(new_n221));
  aoai13aa1n04x5               g126(.a(new_n221), .b(new_n197), .c(new_n159), .d(new_n150), .o1(new_n222));
  xroi22aa1d06x4               g127(.a(new_n217), .b(\b[16] ), .c(new_n216), .d(\b[17] ), .out0(new_n223));
  aoai13aa1n06x5               g128(.a(new_n223), .b(new_n222), .c(new_n175), .d(new_n198), .o1(new_n224));
  nand02aa1d16x5               g129(.a(new_n218), .b(new_n217), .o1(new_n225));
  oaoi03aa1n12x5               g130(.a(\a[18] ), .b(\b[17] ), .c(new_n225), .o1(new_n226));
  inv000aa1d42x5               g131(.a(new_n226), .o1(new_n227));
  nor042aa1n04x5               g132(.a(\b[18] ), .b(\a[19] ), .o1(new_n228));
  nand02aa1d24x5               g133(.a(\b[18] ), .b(\a[19] ), .o1(new_n229));
  norb02aa1n06x4               g134(.a(new_n229), .b(new_n228), .out0(new_n230));
  xnbna2aa1n03x5               g135(.a(new_n230), .b(new_n224), .c(new_n227), .out0(\s[19] ));
  xnrc02aa1n02x5               g136(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nanp02aa1n06x5               g137(.a(new_n224), .b(new_n227), .o1(new_n233));
  xnrc02aa1n12x5               g138(.a(\b[19] ), .b(\a[20] ), .out0(new_n234));
  aoai13aa1n02x5               g139(.a(new_n234), .b(new_n228), .c(new_n233), .d(new_n229), .o1(new_n235));
  aoi112aa1n02x5               g140(.a(new_n228), .b(new_n234), .c(new_n233), .d(new_n229), .o1(new_n236));
  nanb02aa1n03x5               g141(.a(new_n236), .b(new_n235), .out0(\s[20] ));
  norb02aa1n12x5               g142(.a(new_n230), .b(new_n234), .out0(new_n238));
  and002aa1n02x5               g143(.a(new_n223), .b(new_n238), .o(new_n239));
  and002aa1n02x5               g144(.a(\b[19] ), .b(\a[20] ), .o(new_n240));
  oai012aa1n02x5               g145(.a(new_n229), .b(\b[19] ), .c(\a[20] ), .o1(new_n241));
  nona32aa1n09x5               g146(.a(new_n226), .b(new_n241), .c(new_n240), .d(new_n228), .out0(new_n242));
  orn002aa1n02x5               g147(.a(\a[19] ), .b(\b[18] ), .o(new_n243));
  oao003aa1n02x5               g148(.a(\a[20] ), .b(\b[19] ), .c(new_n243), .carry(new_n244));
  nanp02aa1n02x5               g149(.a(new_n242), .b(new_n244), .o1(new_n245));
  tech160nm_fixorc02aa1n03p5x5 g150(.a(\a[21] ), .b(\b[20] ), .out0(new_n246));
  aoai13aa1n06x5               g151(.a(new_n246), .b(new_n245), .c(new_n210), .d(new_n239), .o1(new_n247));
  nanb03aa1n02x5               g152(.a(new_n246), .b(new_n242), .c(new_n244), .out0(new_n248));
  aoi012aa1n02x5               g153(.a(new_n248), .b(new_n210), .c(new_n239), .o1(new_n249));
  norb02aa1n02x5               g154(.a(new_n247), .b(new_n249), .out0(\s[21] ));
  tech160nm_fixnrc02aa1n05x5   g155(.a(\b[21] ), .b(\a[22] ), .out0(new_n251));
  inv000aa1d42x5               g156(.a(\a[21] ), .o1(new_n252));
  oaib12aa1n06x5               g157(.a(new_n247), .b(\b[20] ), .c(new_n252), .out0(new_n253));
  nand02aa1n02x5               g158(.a(new_n253), .b(new_n251), .o1(new_n254));
  norp02aa1n02x5               g159(.a(\b[20] ), .b(\a[21] ), .o1(new_n255));
  nona22aa1n02x5               g160(.a(new_n247), .b(new_n251), .c(new_n255), .out0(new_n256));
  nanp02aa1n03x5               g161(.a(new_n254), .b(new_n256), .o1(\s[22] ));
  inv000aa1d42x5               g162(.a(\a[22] ), .o1(new_n258));
  xroi22aa1d04x5               g163(.a(new_n252), .b(\b[20] ), .c(new_n258), .d(\b[21] ), .out0(new_n259));
  nand23aa1d12x5               g164(.a(new_n259), .b(new_n223), .c(new_n238), .o1(new_n260));
  inv000aa1d42x5               g165(.a(new_n260), .o1(new_n261));
  nanb02aa1n03x5               g166(.a(new_n251), .b(new_n246), .out0(new_n262));
  oai022aa1n02x5               g167(.a(\a[21] ), .b(\b[20] ), .c(\b[21] ), .d(\a[22] ), .o1(new_n263));
  oaib12aa1n02x5               g168(.a(new_n263), .b(new_n258), .c(\b[21] ), .out0(new_n264));
  aoai13aa1n12x5               g169(.a(new_n264), .b(new_n262), .c(new_n242), .d(new_n244), .o1(new_n265));
  ao0012aa1n03x7               g170(.a(new_n265), .b(new_n210), .c(new_n261), .o(new_n266));
  xorb03aa1n02x5               g171(.a(new_n266), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  nor042aa1n06x5               g172(.a(\b[22] ), .b(\a[23] ), .o1(new_n268));
  inv000aa1d42x5               g173(.a(new_n268), .o1(new_n269));
  xorc02aa1n12x5               g174(.a(\a[23] ), .b(\b[22] ), .out0(new_n270));
  aoai13aa1n03x5               g175(.a(new_n270), .b(new_n265), .c(new_n210), .d(new_n261), .o1(new_n271));
  xorc02aa1n12x5               g176(.a(\a[24] ), .b(\b[23] ), .out0(new_n272));
  inv000aa1d42x5               g177(.a(new_n272), .o1(new_n273));
  aob012aa1n03x5               g178(.a(new_n273), .b(new_n271), .c(new_n269), .out0(new_n274));
  nona22aa1n02x5               g179(.a(new_n271), .b(new_n273), .c(new_n268), .out0(new_n275));
  nanp02aa1n03x5               g180(.a(new_n274), .b(new_n275), .o1(\s[24] ));
  and002aa1n06x5               g181(.a(new_n272), .b(new_n270), .o(new_n277));
  oaoi03aa1n02x5               g182(.a(\a[24] ), .b(\b[23] ), .c(new_n269), .o1(new_n278));
  tech160nm_fiaoi012aa1n05x5   g183(.a(new_n278), .b(new_n265), .c(new_n277), .o1(new_n279));
  inv000aa1d42x5               g184(.a(new_n270), .o1(new_n280));
  nona32aa1n09x5               g185(.a(new_n210), .b(new_n273), .c(new_n280), .d(new_n260), .out0(new_n281));
  xorc02aa1n12x5               g186(.a(\a[25] ), .b(\b[24] ), .out0(new_n282));
  xnbna2aa1n03x5               g187(.a(new_n282), .b(new_n281), .c(new_n279), .out0(\s[25] ));
  tech160nm_finand02aa1n03p5x5 g188(.a(new_n281), .b(new_n279), .o1(new_n284));
  nor042aa1n04x5               g189(.a(\b[24] ), .b(\a[25] ), .o1(new_n285));
  tech160nm_fixnrc02aa1n05x5   g190(.a(\b[25] ), .b(\a[26] ), .out0(new_n286));
  aoai13aa1n03x5               g191(.a(new_n286), .b(new_n285), .c(new_n284), .d(new_n282), .o1(new_n287));
  oaoi13aa1n02x5               g192(.a(new_n260), .b(new_n209), .c(new_n200), .d(new_n202), .o1(new_n288));
  inv000aa1n03x5               g193(.a(new_n279), .o1(new_n289));
  aoai13aa1n03x5               g194(.a(new_n282), .b(new_n289), .c(new_n288), .d(new_n277), .o1(new_n290));
  nona22aa1n02x5               g195(.a(new_n290), .b(new_n286), .c(new_n285), .out0(new_n291));
  nanp02aa1n03x5               g196(.a(new_n287), .b(new_n291), .o1(\s[26] ));
  norb02aa1n06x5               g197(.a(new_n282), .b(new_n286), .out0(new_n293));
  aoai13aa1n09x5               g198(.a(new_n293), .b(new_n278), .c(new_n265), .d(new_n277), .o1(new_n294));
  nano22aa1n06x5               g199(.a(new_n260), .b(new_n277), .c(new_n293), .out0(new_n295));
  aoai13aa1n06x5               g200(.a(new_n295), .b(new_n222), .c(new_n175), .d(new_n198), .o1(new_n296));
  inv000aa1n03x5               g201(.a(new_n285), .o1(new_n297));
  oaoi03aa1n12x5               g202(.a(\a[26] ), .b(\b[25] ), .c(new_n297), .o1(new_n298));
  inv000aa1d42x5               g203(.a(new_n298), .o1(new_n299));
  nanp03aa1d12x5               g204(.a(new_n294), .b(new_n296), .c(new_n299), .o1(new_n300));
  xorc02aa1n12x5               g205(.a(\a[27] ), .b(\b[26] ), .out0(new_n301));
  aoi112aa1n02x5               g206(.a(new_n301), .b(new_n298), .c(new_n210), .d(new_n295), .o1(new_n302));
  aoi022aa1n02x5               g207(.a(new_n300), .b(new_n301), .c(new_n302), .d(new_n294), .o1(\s[27] ));
  norp02aa1n02x5               g208(.a(\b[26] ), .b(\a[27] ), .o1(new_n304));
  nor002aa1n02x5               g209(.a(\b[27] ), .b(\a[28] ), .o1(new_n305));
  nanp02aa1n04x5               g210(.a(\b[27] ), .b(\a[28] ), .o1(new_n306));
  nanb02aa1n06x5               g211(.a(new_n305), .b(new_n306), .out0(new_n307));
  aoai13aa1n03x5               g212(.a(new_n307), .b(new_n304), .c(new_n300), .d(new_n301), .o1(new_n308));
  nand02aa1n02x5               g213(.a(new_n300), .b(new_n301), .o1(new_n309));
  nona22aa1n03x5               g214(.a(new_n309), .b(new_n307), .c(new_n304), .out0(new_n310));
  nanp02aa1n03x5               g215(.a(new_n310), .b(new_n308), .o1(\s[28] ));
  norb02aa1n03x5               g216(.a(new_n301), .b(new_n307), .out0(new_n312));
  nand02aa1n02x5               g217(.a(new_n300), .b(new_n312), .o1(new_n313));
  tech160nm_fiaoi012aa1n04x5   g218(.a(new_n298), .b(new_n210), .c(new_n295), .o1(new_n314));
  inv000aa1d42x5               g219(.a(new_n312), .o1(new_n315));
  aoi012aa1n02x5               g220(.a(new_n305), .b(new_n304), .c(new_n306), .o1(new_n316));
  aoai13aa1n02x7               g221(.a(new_n316), .b(new_n315), .c(new_n314), .d(new_n294), .o1(new_n317));
  xorc02aa1n12x5               g222(.a(\a[29] ), .b(\b[28] ), .out0(new_n318));
  norb02aa1n02x5               g223(.a(new_n316), .b(new_n318), .out0(new_n319));
  aoi022aa1n03x5               g224(.a(new_n317), .b(new_n318), .c(new_n313), .d(new_n319), .o1(\s[29] ));
  xorb03aa1n02x5               g225(.a(new_n106), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n02x4               g226(.a(new_n307), .b(new_n301), .c(new_n318), .out0(new_n322));
  nand02aa1n02x5               g227(.a(new_n300), .b(new_n322), .o1(new_n323));
  inv000aa1n02x5               g228(.a(new_n322), .o1(new_n324));
  oao003aa1n02x5               g229(.a(\a[29] ), .b(\b[28] ), .c(new_n316), .carry(new_n325));
  aoai13aa1n02x7               g230(.a(new_n325), .b(new_n324), .c(new_n314), .d(new_n294), .o1(new_n326));
  xorc02aa1n02x5               g231(.a(\a[30] ), .b(\b[29] ), .out0(new_n327));
  norb02aa1n02x5               g232(.a(new_n325), .b(new_n327), .out0(new_n328));
  aoi022aa1n03x5               g233(.a(new_n326), .b(new_n327), .c(new_n323), .d(new_n328), .o1(\s[30] ));
  nand03aa1n02x5               g234(.a(new_n312), .b(new_n318), .c(new_n327), .o1(new_n330));
  nanb02aa1n03x5               g235(.a(new_n330), .b(new_n300), .out0(new_n331));
  xorc02aa1n02x5               g236(.a(\a[31] ), .b(\b[30] ), .out0(new_n332));
  and002aa1n02x5               g237(.a(\b[29] ), .b(\a[30] ), .o(new_n333));
  oabi12aa1n02x5               g238(.a(new_n332), .b(\a[30] ), .c(\b[29] ), .out0(new_n334));
  oab012aa1n02x4               g239(.a(new_n334), .b(new_n325), .c(new_n333), .out0(new_n335));
  oao003aa1n02x5               g240(.a(\a[30] ), .b(\b[29] ), .c(new_n325), .carry(new_n336));
  aoai13aa1n02x7               g241(.a(new_n336), .b(new_n330), .c(new_n314), .d(new_n294), .o1(new_n337));
  aoi022aa1n03x5               g242(.a(new_n337), .b(new_n332), .c(new_n331), .d(new_n335), .o1(\s[31] ));
  xnbna2aa1n03x5               g243(.a(new_n103), .b(new_n165), .c(new_n104), .out0(\s[3] ));
  xorc02aa1n02x5               g244(.a(\a[4] ), .b(\b[3] ), .out0(new_n340));
  aoi113aa1n02x5               g245(.a(new_n340), .b(new_n101), .c(new_n165), .d(new_n102), .e(new_n104), .o1(new_n341));
  aoi022aa1n06x5               g246(.a(new_n166), .b(new_n108), .c(\b[3] ), .d(\a[4] ), .o1(new_n342));
  oaoi13aa1n02x5               g247(.a(new_n341), .b(new_n342), .c(\a[4] ), .d(\b[3] ), .o1(\s[4] ));
  xorb03aa1n02x5               g248(.a(new_n342), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  norb02aa1n02x5               g249(.a(new_n123), .b(new_n117), .out0(new_n345));
  aoai13aa1n03x5               g250(.a(new_n345), .b(new_n110), .c(new_n342), .d(new_n167), .o1(new_n346));
  aoi112aa1n02x5               g251(.a(new_n345), .b(new_n110), .c(new_n342), .d(new_n167), .o1(new_n347));
  norb02aa1n02x5               g252(.a(new_n346), .b(new_n347), .out0(\s[6] ));
  nanb02aa1n02x5               g253(.a(new_n117), .b(new_n346), .out0(new_n349));
  xorb03aa1n02x5               g254(.a(new_n349), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  norb02aa1n02x5               g255(.a(new_n121), .b(new_n113), .out0(new_n351));
  aoai13aa1n02x5               g256(.a(new_n351), .b(new_n116), .c(new_n349), .d(new_n124), .o1(new_n352));
  aoi112aa1n02x5               g257(.a(new_n351), .b(new_n116), .c(new_n349), .d(new_n124), .o1(new_n353));
  norb02aa1n03x4               g258(.a(new_n352), .b(new_n353), .out0(\s[8] ));
  xorb03aa1n02x5               g259(.a(new_n200), .b(\b[8] ), .c(new_n99), .out0(\s[9] ));
endmodule


