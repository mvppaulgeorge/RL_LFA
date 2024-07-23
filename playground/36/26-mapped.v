// Benchmark "adder" written by ABC on Thu Jul 18 06:37:53 2024

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
    new_n125, new_n126, new_n127, new_n128, new_n130, new_n131, new_n132,
    new_n133, new_n134, new_n136, new_n138, new_n139, new_n140, new_n141,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n161, new_n162, new_n163,
    new_n165, new_n166, new_n167, new_n168, new_n169, new_n170, new_n171,
    new_n172, new_n173, new_n174, new_n176, new_n177, new_n178, new_n179,
    new_n180, new_n181, new_n182, new_n183, new_n184, new_n185, new_n186,
    new_n187, new_n188, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n197, new_n198, new_n199, new_n200, new_n201,
    new_n202, new_n204, new_n205, new_n206, new_n207, new_n208, new_n209,
    new_n210, new_n212, new_n213, new_n214, new_n215, new_n216, new_n217,
    new_n218, new_n221, new_n222, new_n223, new_n224, new_n225, new_n226,
    new_n227, new_n228, new_n229, new_n230, new_n231, new_n232, new_n234,
    new_n235, new_n236, new_n237, new_n238, new_n239, new_n240, new_n241,
    new_n243, new_n244, new_n245, new_n246, new_n247, new_n248, new_n249,
    new_n250, new_n251, new_n252, new_n253, new_n254, new_n255, new_n256,
    new_n258, new_n259, new_n260, new_n261, new_n262, new_n263, new_n264,
    new_n265, new_n266, new_n267, new_n269, new_n270, new_n271, new_n272,
    new_n273, new_n274, new_n275, new_n276, new_n277, new_n279, new_n280,
    new_n281, new_n282, new_n283, new_n284, new_n285, new_n286, new_n287,
    new_n288, new_n289, new_n290, new_n291, new_n293, new_n294, new_n295,
    new_n296, new_n297, new_n298, new_n299, new_n300, new_n302, new_n303,
    new_n304, new_n305, new_n306, new_n307, new_n308, new_n309, new_n311,
    new_n312, new_n313, new_n314, new_n315, new_n316, new_n317, new_n318,
    new_n319, new_n321, new_n322, new_n323, new_n324, new_n325, new_n326,
    new_n327, new_n330, new_n331, new_n332, new_n333, new_n334, new_n335,
    new_n336, new_n338, new_n339, new_n340, new_n341, new_n342, new_n343,
    new_n344, new_n345, new_n348, new_n351, new_n353, new_n354, new_n356;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  norp02aa1n02x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(\a[3] ), .o1(new_n98));
  nanb02aa1d24x5               g003(.a(\b[2] ), .b(new_n98), .out0(new_n99));
  oaoi03aa1n09x5               g004(.a(\a[4] ), .b(\b[3] ), .c(new_n99), .o1(new_n100));
  inv000aa1d42x5               g005(.a(new_n100), .o1(new_n101));
  nanp02aa1n02x5               g006(.a(\b[3] ), .b(\a[4] ), .o1(new_n102));
  norp02aa1n02x5               g007(.a(\b[3] ), .b(\a[4] ), .o1(new_n103));
  norb02aa1n02x5               g008(.a(new_n102), .b(new_n103), .out0(new_n104));
  nor042aa1n06x5               g009(.a(\b[1] ), .b(\a[2] ), .o1(new_n105));
  inv000aa1d42x5               g010(.a(new_n105), .o1(new_n106));
  nanp02aa1n02x5               g011(.a(\b[0] ), .b(\a[1] ), .o1(new_n107));
  nanp02aa1n02x5               g012(.a(\b[1] ), .b(\a[2] ), .o1(new_n108));
  aob012aa1n02x5               g013(.a(new_n106), .b(new_n107), .c(new_n108), .out0(new_n109));
  norp02aa1n02x5               g014(.a(\b[2] ), .b(\a[3] ), .o1(new_n110));
  nanp02aa1n02x5               g015(.a(\b[2] ), .b(\a[3] ), .o1(new_n111));
  norb02aa1n02x5               g016(.a(new_n111), .b(new_n110), .out0(new_n112));
  nanp03aa1n02x5               g017(.a(new_n109), .b(new_n104), .c(new_n112), .o1(new_n113));
  xorc02aa1n02x5               g018(.a(\a[6] ), .b(\b[5] ), .out0(new_n114));
  xorc02aa1n02x5               g019(.a(\a[5] ), .b(\b[4] ), .out0(new_n115));
  nanp02aa1n02x5               g020(.a(\b[6] ), .b(\a[7] ), .o1(new_n116));
  nor042aa1n02x5               g021(.a(\b[7] ), .b(\a[8] ), .o1(new_n117));
  nanp02aa1n02x5               g022(.a(\b[7] ), .b(\a[8] ), .o1(new_n118));
  nor042aa1n02x5               g023(.a(\b[6] ), .b(\a[7] ), .o1(new_n119));
  nano23aa1n03x7               g024(.a(new_n119), .b(new_n117), .c(new_n116), .d(new_n118), .out0(new_n120));
  nanp03aa1n02x5               g025(.a(new_n120), .b(new_n114), .c(new_n115), .o1(new_n121));
  orn002aa1n02x5               g026(.a(\a[5] ), .b(\b[4] ), .o(new_n122));
  oaoi03aa1n02x5               g027(.a(\a[6] ), .b(\b[5] ), .c(new_n122), .o1(new_n123));
  oai012aa1n02x5               g028(.a(new_n118), .b(new_n119), .c(new_n117), .o1(new_n124));
  aobi12aa1n06x5               g029(.a(new_n124), .b(new_n120), .c(new_n123), .out0(new_n125));
  aoai13aa1n06x5               g030(.a(new_n125), .b(new_n121), .c(new_n113), .d(new_n101), .o1(new_n126));
  nand42aa1n03x5               g031(.a(\b[8] ), .b(\a[9] ), .o1(new_n127));
  aoi012aa1n02x5               g032(.a(new_n97), .b(new_n126), .c(new_n127), .o1(new_n128));
  xnrb03aa1n02x5               g033(.a(new_n128), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  nand42aa1n03x5               g034(.a(\b[9] ), .b(\a[10] ), .o1(new_n130));
  oai022aa1n02x5               g035(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n131));
  norp02aa1n02x5               g036(.a(\b[9] ), .b(\a[10] ), .o1(new_n132));
  nano23aa1n03x5               g037(.a(new_n97), .b(new_n132), .c(new_n130), .d(new_n127), .out0(new_n133));
  aoi022aa1n02x5               g038(.a(new_n126), .b(new_n133), .c(new_n130), .d(new_n131), .o1(new_n134));
  xnrb03aa1n02x5               g039(.a(new_n134), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  oaoi03aa1n02x5               g040(.a(\a[11] ), .b(\b[10] ), .c(new_n134), .o1(new_n136));
  xorb03aa1n02x5               g041(.a(new_n136), .b(\b[11] ), .c(\a[12] ), .out0(\s[12] ));
  inv000aa1d42x5               g042(.a(\a[4] ), .o1(new_n138));
  inv000aa1d42x5               g043(.a(\b[3] ), .o1(new_n139));
  nanp02aa1n02x5               g044(.a(new_n139), .b(new_n138), .o1(new_n140));
  nanp02aa1n02x5               g045(.a(new_n140), .b(new_n102), .o1(new_n141));
  tech160nm_fiaoi012aa1n04x5   g046(.a(new_n105), .b(new_n107), .c(new_n108), .o1(new_n142));
  nanp02aa1n02x5               g047(.a(new_n99), .b(new_n111), .o1(new_n143));
  norp03aa1n02x5               g048(.a(new_n142), .b(new_n143), .c(new_n141), .o1(new_n144));
  xnrc02aa1n02x5               g049(.a(\b[5] ), .b(\a[6] ), .out0(new_n145));
  xnrc02aa1n02x5               g050(.a(\b[4] ), .b(\a[5] ), .out0(new_n146));
  nona23aa1n02x4               g051(.a(new_n118), .b(new_n116), .c(new_n119), .d(new_n117), .out0(new_n147));
  nor003aa1n03x5               g052(.a(new_n147), .b(new_n146), .c(new_n145), .o1(new_n148));
  oai012aa1n03x5               g053(.a(new_n148), .b(new_n144), .c(new_n100), .o1(new_n149));
  norp02aa1n02x5               g054(.a(\b[10] ), .b(\a[11] ), .o1(new_n150));
  nanp02aa1n02x5               g055(.a(\b[10] ), .b(\a[11] ), .o1(new_n151));
  norp02aa1n02x5               g056(.a(\b[11] ), .b(\a[12] ), .o1(new_n152));
  nanp02aa1n02x5               g057(.a(\b[11] ), .b(\a[12] ), .o1(new_n153));
  nano23aa1n02x5               g058(.a(new_n150), .b(new_n152), .c(new_n153), .d(new_n151), .out0(new_n154));
  nand02aa1d04x5               g059(.a(new_n154), .b(new_n133), .o1(new_n155));
  nanp03aa1n02x5               g060(.a(new_n131), .b(new_n130), .c(new_n151), .o1(new_n156));
  norp02aa1n02x5               g061(.a(new_n152), .b(new_n150), .o1(new_n157));
  aob012aa1n02x5               g062(.a(new_n153), .b(new_n156), .c(new_n157), .out0(new_n158));
  aoai13aa1n03x5               g063(.a(new_n158), .b(new_n155), .c(new_n149), .d(new_n125), .o1(new_n159));
  xorb03aa1n02x5               g064(.a(new_n159), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  nor022aa1n06x5               g065(.a(\b[12] ), .b(\a[13] ), .o1(new_n161));
  nanp02aa1n02x5               g066(.a(\b[12] ), .b(\a[13] ), .o1(new_n162));
  aoi012aa1n02x5               g067(.a(new_n161), .b(new_n159), .c(new_n162), .o1(new_n163));
  xnrb03aa1n02x5               g068(.a(new_n163), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  oai013aa1n03x5               g069(.a(new_n101), .b(new_n142), .c(new_n143), .d(new_n141), .o1(new_n165));
  oaib12aa1n02x5               g070(.a(new_n124), .b(new_n147), .c(new_n123), .out0(new_n166));
  inv000aa1d42x5               g071(.a(new_n155), .o1(new_n167));
  aoai13aa1n02x5               g072(.a(new_n167), .b(new_n166), .c(new_n165), .d(new_n148), .o1(new_n168));
  nor022aa1n08x5               g073(.a(\b[13] ), .b(\a[14] ), .o1(new_n169));
  nanp02aa1n02x5               g074(.a(\b[13] ), .b(\a[14] ), .o1(new_n170));
  nona23aa1n03x5               g075(.a(new_n170), .b(new_n162), .c(new_n161), .d(new_n169), .out0(new_n171));
  oa0012aa1n02x5               g076(.a(new_n170), .b(new_n169), .c(new_n161), .o(new_n172));
  inv000aa1d42x5               g077(.a(new_n172), .o1(new_n173));
  aoai13aa1n02x5               g078(.a(new_n173), .b(new_n171), .c(new_n168), .d(new_n158), .o1(new_n174));
  xorb03aa1n02x5               g079(.a(new_n174), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  inv040aa1d30x5               g080(.a(\a[15] ), .o1(new_n176));
  inv000aa1d42x5               g081(.a(\b[14] ), .o1(new_n177));
  nand02aa1d28x5               g082(.a(new_n177), .b(new_n176), .o1(new_n178));
  nano23aa1n06x5               g083(.a(new_n161), .b(new_n169), .c(new_n170), .d(new_n162), .out0(new_n179));
  nand02aa1n03x5               g084(.a(\b[14] ), .b(\a[15] ), .o1(new_n180));
  nand02aa1n08x5               g085(.a(new_n178), .b(new_n180), .o1(new_n181));
  inv000aa1d42x5               g086(.a(new_n181), .o1(new_n182));
  aoai13aa1n02x5               g087(.a(new_n182), .b(new_n172), .c(new_n159), .d(new_n179), .o1(new_n183));
  xnrc02aa1n02x5               g088(.a(\b[15] ), .b(\a[16] ), .out0(new_n184));
  tech160nm_fiaoi012aa1n03p5x5 g089(.a(new_n184), .b(new_n183), .c(new_n178), .o1(new_n185));
  inv000aa1d42x5               g090(.a(new_n178), .o1(new_n186));
  xorc02aa1n02x5               g091(.a(\a[16] ), .b(\b[15] ), .out0(new_n187));
  aoi112aa1n02x5               g092(.a(new_n186), .b(new_n187), .c(new_n174), .d(new_n180), .o1(new_n188));
  norp02aa1n02x5               g093(.a(new_n185), .b(new_n188), .o1(\s[16] ));
  nanp03aa1n02x5               g094(.a(new_n179), .b(new_n182), .c(new_n187), .o1(new_n190));
  nor042aa1n06x5               g095(.a(new_n190), .b(new_n155), .o1(new_n191));
  aoai13aa1n09x5               g096(.a(new_n191), .b(new_n166), .c(new_n165), .d(new_n148), .o1(new_n192));
  aoi022aa1n02x5               g097(.a(new_n156), .b(new_n157), .c(\b[11] ), .d(\a[12] ), .o1(new_n193));
  nor003aa1n03x5               g098(.a(new_n171), .b(new_n181), .c(new_n184), .o1(new_n194));
  inv000aa1d42x5               g099(.a(\a[16] ), .o1(new_n195));
  inv000aa1d42x5               g100(.a(\b[15] ), .o1(new_n196));
  nanp02aa1n02x5               g101(.a(new_n196), .b(new_n195), .o1(new_n197));
  and002aa1n02x5               g102(.a(\b[15] ), .b(\a[16] ), .o(new_n198));
  oai112aa1n03x5               g103(.a(new_n170), .b(new_n180), .c(new_n169), .d(new_n161), .o1(new_n199));
  aoai13aa1n02x5               g104(.a(new_n197), .b(new_n198), .c(new_n199), .d(new_n178), .o1(new_n200));
  aoi012aa1n09x5               g105(.a(new_n200), .b(new_n194), .c(new_n193), .o1(new_n201));
  xorc02aa1n12x5               g106(.a(\a[17] ), .b(\b[16] ), .out0(new_n202));
  xnbna2aa1n03x5               g107(.a(new_n202), .b(new_n192), .c(new_n201), .out0(\s[17] ));
  nor002aa1n12x5               g108(.a(\b[16] ), .b(\a[17] ), .o1(new_n204));
  inv000aa1d42x5               g109(.a(new_n204), .o1(new_n205));
  nanp02aa1n02x5               g110(.a(new_n199), .b(new_n178), .o1(new_n206));
  oaoi03aa1n02x5               g111(.a(new_n195), .b(new_n196), .c(new_n206), .o1(new_n207));
  oai012aa1n02x5               g112(.a(new_n207), .b(new_n158), .c(new_n190), .o1(new_n208));
  aoai13aa1n02x5               g113(.a(new_n202), .b(new_n208), .c(new_n126), .d(new_n191), .o1(new_n209));
  xnrc02aa1n02x5               g114(.a(\b[17] ), .b(\a[18] ), .out0(new_n210));
  xobna2aa1n03x5               g115(.a(new_n210), .b(new_n209), .c(new_n205), .out0(\s[18] ));
  inv000aa1d42x5               g116(.a(\a[17] ), .o1(new_n212));
  inv000aa1d42x5               g117(.a(\a[18] ), .o1(new_n213));
  xroi22aa1d04x5               g118(.a(new_n212), .b(\b[16] ), .c(new_n213), .d(\b[17] ), .out0(new_n214));
  inv000aa1d42x5               g119(.a(new_n214), .o1(new_n215));
  oaoi03aa1n02x5               g120(.a(\a[18] ), .b(\b[17] ), .c(new_n205), .o1(new_n216));
  inv000aa1d42x5               g121(.a(new_n216), .o1(new_n217));
  aoai13aa1n06x5               g122(.a(new_n217), .b(new_n215), .c(new_n192), .d(new_n201), .o1(new_n218));
  xorb03aa1n02x5               g123(.a(new_n218), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g124(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor022aa1n16x5               g125(.a(\b[18] ), .b(\a[19] ), .o1(new_n221));
  nand42aa1n06x5               g126(.a(\b[18] ), .b(\a[19] ), .o1(new_n222));
  tech160nm_fixnrc02aa1n04x5   g127(.a(\b[19] ), .b(\a[20] ), .out0(new_n223));
  inv030aa1n02x5               g128(.a(new_n223), .o1(new_n224));
  aoi112aa1n03x4               g129(.a(new_n221), .b(new_n224), .c(new_n218), .d(new_n222), .o1(new_n225));
  inv000aa1d42x5               g130(.a(new_n221), .o1(new_n226));
  nanp03aa1n02x5               g131(.a(new_n194), .b(new_n133), .c(new_n154), .o1(new_n227));
  aoai13aa1n06x5               g132(.a(new_n201), .b(new_n227), .c(new_n149), .d(new_n125), .o1(new_n228));
  nanb02aa1d24x5               g133(.a(new_n221), .b(new_n222), .out0(new_n229));
  inv000aa1d42x5               g134(.a(new_n229), .o1(new_n230));
  aoai13aa1n03x5               g135(.a(new_n230), .b(new_n216), .c(new_n228), .d(new_n214), .o1(new_n231));
  tech160nm_fiaoi012aa1n03p5x5 g136(.a(new_n223), .b(new_n231), .c(new_n226), .o1(new_n232));
  nor002aa1n02x5               g137(.a(new_n232), .b(new_n225), .o1(\s[20] ));
  nona23aa1d18x5               g138(.a(new_n224), .b(new_n202), .c(new_n210), .d(new_n229), .out0(new_n234));
  nor002aa1n02x5               g139(.a(\b[17] ), .b(\a[18] ), .o1(new_n235));
  nanp02aa1n02x5               g140(.a(\b[17] ), .b(\a[18] ), .o1(new_n236));
  oai112aa1n04x5               g141(.a(new_n236), .b(new_n222), .c(new_n235), .d(new_n204), .o1(new_n237));
  oab012aa1n04x5               g142(.a(new_n221), .b(\a[20] ), .c(\b[19] ), .out0(new_n238));
  aoi022aa1n12x5               g143(.a(new_n237), .b(new_n238), .c(\b[19] ), .d(\a[20] ), .o1(new_n239));
  inv000aa1d42x5               g144(.a(new_n239), .o1(new_n240));
  aoai13aa1n04x5               g145(.a(new_n240), .b(new_n234), .c(new_n192), .d(new_n201), .o1(new_n241));
  xorb03aa1n02x5               g146(.a(new_n241), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor042aa1n04x5               g147(.a(\b[20] ), .b(\a[21] ), .o1(new_n243));
  nanp02aa1n02x5               g148(.a(\b[20] ), .b(\a[21] ), .o1(new_n244));
  norb02aa1n02x5               g149(.a(new_n244), .b(new_n243), .out0(new_n245));
  inv000aa1d42x5               g150(.a(\a[22] ), .o1(new_n246));
  inv000aa1d42x5               g151(.a(\b[21] ), .o1(new_n247));
  nand22aa1n03x5               g152(.a(new_n247), .b(new_n246), .o1(new_n248));
  nanp02aa1n04x5               g153(.a(\b[21] ), .b(\a[22] ), .o1(new_n249));
  nand02aa1n08x5               g154(.a(new_n248), .b(new_n249), .o1(new_n250));
  inv000aa1d42x5               g155(.a(new_n250), .o1(new_n251));
  aoi112aa1n03x4               g156(.a(new_n243), .b(new_n251), .c(new_n241), .d(new_n245), .o1(new_n252));
  inv030aa1n02x5               g157(.a(new_n243), .o1(new_n253));
  inv000aa1d42x5               g158(.a(new_n234), .o1(new_n254));
  aoai13aa1n02x5               g159(.a(new_n245), .b(new_n239), .c(new_n228), .d(new_n254), .o1(new_n255));
  tech160nm_fiaoi012aa1n02p5x5 g160(.a(new_n250), .b(new_n255), .c(new_n253), .o1(new_n256));
  norp02aa1n03x5               g161(.a(new_n256), .b(new_n252), .o1(\s[22] ));
  nano22aa1n03x7               g162(.a(new_n250), .b(new_n253), .c(new_n244), .out0(new_n258));
  nano32aa1n02x4               g163(.a(new_n215), .b(new_n258), .c(new_n230), .d(new_n224), .out0(new_n259));
  inv000aa1n02x5               g164(.a(new_n259), .o1(new_n260));
  tech160nm_finand02aa1n03p5x5 g165(.a(new_n237), .b(new_n238), .o1(new_n261));
  nanp02aa1n02x5               g166(.a(\b[19] ), .b(\a[20] ), .o1(new_n262));
  nano32aa1d12x5               g167(.a(new_n250), .b(new_n253), .c(new_n244), .d(new_n262), .out0(new_n263));
  oaoi03aa1n12x5               g168(.a(new_n246), .b(new_n247), .c(new_n243), .o1(new_n264));
  inv000aa1d42x5               g169(.a(new_n264), .o1(new_n265));
  aoi012aa1n12x5               g170(.a(new_n265), .b(new_n261), .c(new_n263), .o1(new_n266));
  aoai13aa1n04x5               g171(.a(new_n266), .b(new_n260), .c(new_n192), .d(new_n201), .o1(new_n267));
  xorb03aa1n02x5               g172(.a(new_n267), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  nor042aa1n03x5               g173(.a(\b[22] ), .b(\a[23] ), .o1(new_n269));
  xorc02aa1n02x5               g174(.a(\a[23] ), .b(\b[22] ), .out0(new_n270));
  xorc02aa1n12x5               g175(.a(\a[24] ), .b(\b[23] ), .out0(new_n271));
  aoi112aa1n03x4               g176(.a(new_n269), .b(new_n271), .c(new_n267), .d(new_n270), .o1(new_n272));
  inv000aa1d42x5               g177(.a(new_n269), .o1(new_n273));
  inv000aa1d42x5               g178(.a(new_n266), .o1(new_n274));
  aoai13aa1n02x5               g179(.a(new_n270), .b(new_n274), .c(new_n228), .d(new_n259), .o1(new_n275));
  inv000aa1d42x5               g180(.a(new_n271), .o1(new_n276));
  tech160nm_fiaoi012aa1n03p5x5 g181(.a(new_n276), .b(new_n275), .c(new_n273), .o1(new_n277));
  nor002aa1n02x5               g182(.a(new_n277), .b(new_n272), .o1(\s[24] ));
  nano32aa1d12x5               g183(.a(new_n234), .b(new_n271), .c(new_n258), .d(new_n270), .out0(new_n279));
  inv000aa1n02x5               g184(.a(new_n279), .o1(new_n280));
  norp02aa1n02x5               g185(.a(new_n235), .b(new_n204), .o1(new_n281));
  nano22aa1n02x4               g186(.a(new_n281), .b(new_n236), .c(new_n222), .out0(new_n282));
  inv030aa1n03x5               g187(.a(new_n238), .o1(new_n283));
  oai012aa1n06x5               g188(.a(new_n263), .b(new_n282), .c(new_n283), .o1(new_n284));
  xnrc02aa1n02x5               g189(.a(\b[22] ), .b(\a[23] ), .out0(new_n285));
  norb02aa1n03x5               g190(.a(new_n271), .b(new_n285), .out0(new_n286));
  inv040aa1n02x5               g191(.a(new_n286), .o1(new_n287));
  oao003aa1n02x5               g192(.a(\a[24] ), .b(\b[23] ), .c(new_n273), .carry(new_n288));
  aoai13aa1n12x5               g193(.a(new_n288), .b(new_n287), .c(new_n284), .d(new_n264), .o1(new_n289));
  inv000aa1d42x5               g194(.a(new_n289), .o1(new_n290));
  aoai13aa1n04x5               g195(.a(new_n290), .b(new_n280), .c(new_n192), .d(new_n201), .o1(new_n291));
  xorb03aa1n02x5               g196(.a(new_n291), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g197(.a(\b[24] ), .b(\a[25] ), .o1(new_n293));
  xorc02aa1n02x5               g198(.a(\a[25] ), .b(\b[24] ), .out0(new_n294));
  xorc02aa1n12x5               g199(.a(\a[26] ), .b(\b[25] ), .out0(new_n295));
  aoi112aa1n03x4               g200(.a(new_n293), .b(new_n295), .c(new_n291), .d(new_n294), .o1(new_n296));
  inv000aa1n02x5               g201(.a(new_n293), .o1(new_n297));
  aoai13aa1n03x5               g202(.a(new_n294), .b(new_n289), .c(new_n228), .d(new_n279), .o1(new_n298));
  inv000aa1d42x5               g203(.a(new_n295), .o1(new_n299));
  tech160nm_fiaoi012aa1n02p5x5 g204(.a(new_n299), .b(new_n298), .c(new_n297), .o1(new_n300));
  nor002aa1n02x5               g205(.a(new_n300), .b(new_n296), .o1(\s[26] ));
  nanp02aa1n04x5               g206(.a(new_n295), .b(new_n294), .o1(new_n302));
  nano23aa1n09x5               g207(.a(new_n234), .b(new_n302), .c(new_n286), .d(new_n258), .out0(new_n303));
  aoai13aa1n06x5               g208(.a(new_n303), .b(new_n208), .c(new_n126), .d(new_n191), .o1(new_n304));
  inv000aa1d42x5               g209(.a(new_n302), .o1(new_n305));
  oao003aa1n02x5               g210(.a(\a[26] ), .b(\b[25] ), .c(new_n297), .carry(new_n306));
  inv000aa1d42x5               g211(.a(new_n306), .o1(new_n307));
  aoi012aa1d18x5               g212(.a(new_n307), .b(new_n289), .c(new_n305), .o1(new_n308));
  xorc02aa1n02x5               g213(.a(\a[27] ), .b(\b[26] ), .out0(new_n309));
  xnbna2aa1n03x5               g214(.a(new_n309), .b(new_n308), .c(new_n304), .out0(\s[27] ));
  norp02aa1n02x5               g215(.a(\b[26] ), .b(\a[27] ), .o1(new_n311));
  inv040aa1n03x5               g216(.a(new_n311), .o1(new_n312));
  aobi12aa1n03x5               g217(.a(new_n309), .b(new_n308), .c(new_n304), .out0(new_n313));
  xnrc02aa1n02x5               g218(.a(\b[27] ), .b(\a[28] ), .out0(new_n314));
  nano22aa1n03x5               g219(.a(new_n313), .b(new_n312), .c(new_n314), .out0(new_n315));
  aoai13aa1n06x5               g220(.a(new_n286), .b(new_n265), .c(new_n261), .d(new_n263), .o1(new_n316));
  aoai13aa1n06x5               g221(.a(new_n306), .b(new_n302), .c(new_n316), .d(new_n288), .o1(new_n317));
  aoai13aa1n03x5               g222(.a(new_n309), .b(new_n317), .c(new_n228), .d(new_n303), .o1(new_n318));
  tech160nm_fiaoi012aa1n02p5x5 g223(.a(new_n314), .b(new_n318), .c(new_n312), .o1(new_n319));
  nor002aa1n02x5               g224(.a(new_n319), .b(new_n315), .o1(\s[28] ));
  xnrc02aa1n02x5               g225(.a(\b[28] ), .b(\a[29] ), .out0(new_n321));
  norb02aa1n02x5               g226(.a(new_n309), .b(new_n314), .out0(new_n322));
  aoai13aa1n02x5               g227(.a(new_n322), .b(new_n317), .c(new_n228), .d(new_n303), .o1(new_n323));
  oao003aa1n02x5               g228(.a(\a[28] ), .b(\b[27] ), .c(new_n312), .carry(new_n324));
  tech160nm_fiaoi012aa1n02p5x5 g229(.a(new_n321), .b(new_n323), .c(new_n324), .o1(new_n325));
  aobi12aa1n03x5               g230(.a(new_n322), .b(new_n308), .c(new_n304), .out0(new_n326));
  nano22aa1n03x5               g231(.a(new_n326), .b(new_n321), .c(new_n324), .out0(new_n327));
  norp02aa1n02x5               g232(.a(new_n325), .b(new_n327), .o1(\s[29] ));
  xorb03aa1n02x5               g233(.a(new_n107), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g234(.a(new_n309), .b(new_n321), .c(new_n314), .out0(new_n330));
  aoai13aa1n03x5               g235(.a(new_n330), .b(new_n317), .c(new_n228), .d(new_n303), .o1(new_n331));
  oao003aa1n02x5               g236(.a(\a[29] ), .b(\b[28] ), .c(new_n324), .carry(new_n332));
  xnrc02aa1n02x5               g237(.a(\b[29] ), .b(\a[30] ), .out0(new_n333));
  tech160nm_fiaoi012aa1n02p5x5 g238(.a(new_n333), .b(new_n331), .c(new_n332), .o1(new_n334));
  aobi12aa1n03x5               g239(.a(new_n330), .b(new_n308), .c(new_n304), .out0(new_n335));
  nano22aa1n03x5               g240(.a(new_n335), .b(new_n332), .c(new_n333), .out0(new_n336));
  norp02aa1n02x5               g241(.a(new_n334), .b(new_n336), .o1(\s[30] ));
  xnrc02aa1n02x5               g242(.a(\b[30] ), .b(\a[31] ), .out0(new_n338));
  nona32aa1n02x4               g243(.a(new_n309), .b(new_n333), .c(new_n321), .d(new_n314), .out0(new_n339));
  aoi012aa1n03x5               g244(.a(new_n339), .b(new_n308), .c(new_n304), .o1(new_n340));
  oao003aa1n02x5               g245(.a(\a[30] ), .b(\b[29] ), .c(new_n332), .carry(new_n341));
  nano22aa1n03x5               g246(.a(new_n340), .b(new_n338), .c(new_n341), .out0(new_n342));
  inv000aa1d42x5               g247(.a(new_n339), .o1(new_n343));
  aoai13aa1n02x5               g248(.a(new_n343), .b(new_n317), .c(new_n228), .d(new_n303), .o1(new_n344));
  tech160nm_fiaoi012aa1n02p5x5 g249(.a(new_n338), .b(new_n344), .c(new_n341), .o1(new_n345));
  norp02aa1n02x5               g250(.a(new_n345), .b(new_n342), .o1(\s[31] ));
  xnbna2aa1n03x5               g251(.a(new_n142), .b(new_n111), .c(new_n99), .out0(\s[3] ));
  oaoi03aa1n02x5               g252(.a(\a[3] ), .b(\b[2] ), .c(new_n142), .o1(new_n348));
  xorb03aa1n02x5               g253(.a(new_n348), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g254(.a(new_n165), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  aoai13aa1n02x5               g255(.a(new_n122), .b(new_n146), .c(new_n113), .d(new_n101), .o1(new_n351));
  xorb03aa1n02x5               g256(.a(new_n351), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  norp02aa1n02x5               g257(.a(new_n146), .b(new_n145), .o1(new_n353));
  tech160nm_fiao0012aa1n02p5x5 g258(.a(new_n123), .b(new_n165), .c(new_n353), .o(new_n354));
  xorb03aa1n02x5               g259(.a(new_n354), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoi012aa1n02x5               g260(.a(new_n119), .b(new_n354), .c(new_n116), .o1(new_n356));
  xnrb03aa1n02x5               g261(.a(new_n356), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xorb03aa1n02x5               g262(.a(new_n126), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


