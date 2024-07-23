// Benchmark "adder" written by ABC on Wed Jul 17 14:57:16 2024

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
    new_n125, new_n126, new_n127, new_n128, new_n129, new_n130, new_n131,
    new_n132, new_n133, new_n134, new_n136, new_n137, new_n138, new_n139,
    new_n140, new_n141, new_n142, new_n144, new_n145, new_n146, new_n147,
    new_n148, new_n149, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n161, new_n162,
    new_n163, new_n164, new_n165, new_n167, new_n168, new_n169, new_n170,
    new_n171, new_n172, new_n173, new_n174, new_n175, new_n176, new_n178,
    new_n179, new_n180, new_n181, new_n182, new_n184, new_n185, new_n186,
    new_n187, new_n188, new_n189, new_n190, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n197, new_n198, new_n199, new_n200, new_n201,
    new_n203, new_n204, new_n205, new_n206, new_n207, new_n208, new_n209,
    new_n210, new_n211, new_n212, new_n213, new_n214, new_n215, new_n217,
    new_n218, new_n219, new_n220, new_n221, new_n222, new_n223, new_n226,
    new_n227, new_n228, new_n229, new_n230, new_n231, new_n232, new_n233,
    new_n234, new_n235, new_n237, new_n238, new_n239, new_n240, new_n241,
    new_n242, new_n243, new_n244, new_n245, new_n246, new_n247, new_n248,
    new_n249, new_n250, new_n251, new_n253, new_n254, new_n255, new_n256,
    new_n257, new_n258, new_n259, new_n260, new_n262, new_n263, new_n264,
    new_n265, new_n266, new_n267, new_n268, new_n269, new_n270, new_n271,
    new_n273, new_n274, new_n275, new_n276, new_n277, new_n278, new_n279,
    new_n281, new_n282, new_n283, new_n284, new_n285, new_n286, new_n287,
    new_n288, new_n289, new_n290, new_n291, new_n292, new_n294, new_n295,
    new_n296, new_n297, new_n298, new_n299, new_n301, new_n302, new_n303,
    new_n304, new_n305, new_n306, new_n307, new_n308, new_n309, new_n310,
    new_n312, new_n313, new_n314, new_n315, new_n316, new_n317, new_n318,
    new_n320, new_n321, new_n322, new_n323, new_n324, new_n325, new_n326,
    new_n329, new_n330, new_n331, new_n332, new_n333, new_n334, new_n335,
    new_n336, new_n337, new_n338, new_n340, new_n341, new_n342, new_n343,
    new_n344, new_n345, new_n346, new_n347, new_n348, new_n349, new_n350,
    new_n351, new_n353, new_n354, new_n356, new_n357, new_n359, new_n361,
    new_n362, new_n363, new_n365, new_n366, new_n367, new_n369, new_n370,
    new_n372;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor042aa1n09x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(new_n97), .o1(new_n98));
  orn002aa1n02x7               g003(.a(\a[2] ), .b(\b[1] ), .o(new_n99));
  nanp02aa1n02x5               g004(.a(\b[0] ), .b(\a[1] ), .o1(new_n100));
  nanp02aa1n02x5               g005(.a(\b[1] ), .b(\a[2] ), .o1(new_n101));
  aob012aa1n06x5               g006(.a(new_n99), .b(new_n100), .c(new_n101), .out0(new_n102));
  nor042aa1n03x5               g007(.a(\b[3] ), .b(\a[4] ), .o1(new_n103));
  nand02aa1n04x5               g008(.a(\b[3] ), .b(\a[4] ), .o1(new_n104));
  nor042aa1n04x5               g009(.a(\b[2] ), .b(\a[3] ), .o1(new_n105));
  nanp02aa1n02x5               g010(.a(\b[2] ), .b(\a[3] ), .o1(new_n106));
  nano23aa1n06x5               g011(.a(new_n103), .b(new_n105), .c(new_n106), .d(new_n104), .out0(new_n107));
  nanp02aa1n03x5               g012(.a(new_n107), .b(new_n102), .o1(new_n108));
  aoi012aa1n06x5               g013(.a(new_n103), .b(new_n105), .c(new_n104), .o1(new_n109));
  nand02aa1d04x5               g014(.a(new_n108), .b(new_n109), .o1(new_n110));
  aoi022aa1n02x5               g015(.a(\b[7] ), .b(\a[8] ), .c(\a[7] ), .d(\b[6] ), .o1(new_n111));
  nor022aa1n16x5               g016(.a(\b[7] ), .b(\a[8] ), .o1(new_n112));
  nor002aa1d32x5               g017(.a(\b[6] ), .b(\a[7] ), .o1(new_n113));
  nona22aa1n03x5               g018(.a(new_n111), .b(new_n112), .c(new_n113), .out0(new_n114));
  inv000aa1d42x5               g019(.a(\a[6] ), .o1(new_n115));
  inv000aa1d42x5               g020(.a(\b[5] ), .o1(new_n116));
  aoi022aa1n02x5               g021(.a(new_n116), .b(new_n115), .c(\a[5] ), .d(\b[4] ), .o1(new_n117));
  inv000aa1d42x5               g022(.a(\a[5] ), .o1(new_n118));
  inv000aa1d42x5               g023(.a(\b[4] ), .o1(new_n119));
  aoi022aa1n02x7               g024(.a(new_n119), .b(new_n118), .c(\a[6] ), .d(\b[5] ), .o1(new_n120));
  nano22aa1n03x7               g025(.a(new_n114), .b(new_n117), .c(new_n120), .out0(new_n121));
  and002aa1n12x5               g026(.a(\b[7] ), .b(\a[8] ), .o(new_n122));
  nor043aa1n04x5               g027(.a(new_n122), .b(new_n112), .c(new_n113), .o1(new_n123));
  nand42aa1n03x5               g028(.a(new_n116), .b(new_n115), .o1(new_n124));
  nand42aa1n03x5               g029(.a(\b[5] ), .b(\a[6] ), .o1(new_n125));
  oai112aa1n06x5               g030(.a(new_n124), .b(new_n125), .c(\b[4] ), .d(\a[5] ), .o1(new_n126));
  aoi022aa1n06x5               g031(.a(\b[6] ), .b(\a[7] ), .c(\a[6] ), .d(\b[5] ), .o1(new_n127));
  nanp03aa1n03x5               g032(.a(new_n123), .b(new_n126), .c(new_n127), .o1(new_n128));
  tech160nm_fioai012aa1n05x5   g033(.a(new_n128), .b(new_n123), .c(new_n122), .o1(new_n129));
  tech160nm_fixorc02aa1n05x5   g034(.a(\a[9] ), .b(\b[8] ), .out0(new_n130));
  aoai13aa1n06x5               g035(.a(new_n130), .b(new_n129), .c(new_n110), .d(new_n121), .o1(new_n131));
  norp02aa1n04x5               g036(.a(\b[9] ), .b(\a[10] ), .o1(new_n132));
  nand22aa1n06x5               g037(.a(\b[9] ), .b(\a[10] ), .o1(new_n133));
  norb02aa1n06x4               g038(.a(new_n133), .b(new_n132), .out0(new_n134));
  xnbna2aa1n03x5               g039(.a(new_n134), .b(new_n131), .c(new_n98), .out0(\s[10] ));
  nona22aa1n02x4               g040(.a(new_n131), .b(new_n132), .c(new_n97), .out0(new_n136));
  nor002aa1d24x5               g041(.a(\b[10] ), .b(\a[11] ), .o1(new_n137));
  nand02aa1n03x5               g042(.a(\b[10] ), .b(\a[11] ), .o1(new_n138));
  nand02aa1n03x5               g043(.a(new_n138), .b(new_n133), .o1(new_n139));
  nona22aa1n03x5               g044(.a(new_n136), .b(new_n137), .c(new_n139), .out0(new_n140));
  inv000aa1d42x5               g045(.a(new_n137), .o1(new_n141));
  aoi022aa1n02x5               g046(.a(new_n136), .b(new_n133), .c(new_n138), .d(new_n141), .o1(new_n142));
  norb02aa1n02x5               g047(.a(new_n140), .b(new_n142), .out0(\s[11] ));
  nor022aa1n16x5               g048(.a(\b[11] ), .b(\a[12] ), .o1(new_n144));
  nand02aa1n06x5               g049(.a(\b[11] ), .b(\a[12] ), .o1(new_n145));
  norb02aa1n02x5               g050(.a(new_n145), .b(new_n144), .out0(new_n146));
  inv000aa1n02x5               g051(.a(new_n144), .o1(new_n147));
  aoi012aa1n02x5               g052(.a(new_n137), .b(new_n147), .c(new_n145), .o1(new_n148));
  nanp02aa1n02x5               g053(.a(new_n140), .b(new_n141), .o1(new_n149));
  aoi022aa1n02x5               g054(.a(new_n149), .b(new_n146), .c(new_n140), .d(new_n148), .o1(\s[12] ));
  nona23aa1n02x4               g055(.a(new_n145), .b(new_n138), .c(new_n137), .d(new_n144), .out0(new_n151));
  nano22aa1n03x7               g056(.a(new_n151), .b(new_n130), .c(new_n134), .out0(new_n152));
  aoai13aa1n02x5               g057(.a(new_n152), .b(new_n129), .c(new_n110), .d(new_n121), .o1(new_n153));
  norb03aa1n06x5               g058(.a(new_n145), .b(new_n137), .c(new_n144), .out0(new_n154));
  oab012aa1n06x5               g059(.a(new_n139), .b(new_n97), .c(new_n132), .out0(new_n155));
  nand22aa1n03x5               g060(.a(new_n155), .b(new_n154), .o1(new_n156));
  aob012aa1n03x5               g061(.a(new_n147), .b(new_n137), .c(new_n145), .out0(new_n157));
  inv000aa1n02x5               g062(.a(new_n157), .o1(new_n158));
  nand02aa1d04x5               g063(.a(new_n156), .b(new_n158), .o1(new_n159));
  nanb02aa1n02x5               g064(.a(new_n159), .b(new_n153), .out0(new_n160));
  nor002aa1d32x5               g065(.a(\b[12] ), .b(\a[13] ), .o1(new_n161));
  tech160nm_finand02aa1n03p5x5 g066(.a(\b[12] ), .b(\a[13] ), .o1(new_n162));
  nanb02aa1n02x5               g067(.a(new_n161), .b(new_n162), .out0(new_n163));
  inv000aa1d42x5               g068(.a(new_n163), .o1(new_n164));
  aoi112aa1n02x5               g069(.a(new_n157), .b(new_n164), .c(new_n155), .d(new_n154), .o1(new_n165));
  aoi022aa1n02x5               g070(.a(new_n160), .b(new_n164), .c(new_n153), .d(new_n165), .o1(\s[13] ));
  inv000aa1d42x5               g071(.a(new_n161), .o1(new_n167));
  inv020aa1n02x5               g072(.a(new_n109), .o1(new_n168));
  aoai13aa1n06x5               g073(.a(new_n121), .b(new_n168), .c(new_n102), .d(new_n107), .o1(new_n169));
  oab012aa1n02x4               g074(.a(new_n122), .b(new_n112), .c(new_n113), .out0(new_n170));
  aoi013aa1n06x4               g075(.a(new_n170), .b(new_n123), .c(new_n126), .d(new_n127), .o1(new_n171));
  nanp02aa1n03x5               g076(.a(new_n169), .b(new_n171), .o1(new_n172));
  aoai13aa1n03x5               g077(.a(new_n164), .b(new_n159), .c(new_n172), .d(new_n152), .o1(new_n173));
  nor002aa1n20x5               g078(.a(\b[13] ), .b(\a[14] ), .o1(new_n174));
  nand42aa1n08x5               g079(.a(\b[13] ), .b(\a[14] ), .o1(new_n175));
  nanb02aa1n02x5               g080(.a(new_n174), .b(new_n175), .out0(new_n176));
  xobna2aa1n03x5               g081(.a(new_n176), .b(new_n173), .c(new_n167), .out0(\s[14] ));
  nano23aa1n09x5               g082(.a(new_n161), .b(new_n174), .c(new_n175), .d(new_n162), .out0(new_n178));
  aoai13aa1n06x5               g083(.a(new_n178), .b(new_n159), .c(new_n172), .d(new_n152), .o1(new_n179));
  oai012aa1d24x5               g084(.a(new_n175), .b(new_n174), .c(new_n161), .o1(new_n180));
  xnrc02aa1n12x5               g085(.a(\b[14] ), .b(\a[15] ), .out0(new_n181));
  inv000aa1d42x5               g086(.a(new_n181), .o1(new_n182));
  xnbna2aa1n03x5               g087(.a(new_n182), .b(new_n179), .c(new_n180), .out0(\s[15] ));
  inv000aa1d42x5               g088(.a(new_n180), .o1(new_n184));
  aoai13aa1n03x5               g089(.a(new_n182), .b(new_n184), .c(new_n160), .d(new_n178), .o1(new_n185));
  xorc02aa1n12x5               g090(.a(\a[16] ), .b(\b[15] ), .out0(new_n186));
  nor042aa1n06x5               g091(.a(\b[14] ), .b(\a[15] ), .o1(new_n187));
  norp02aa1n02x5               g092(.a(new_n186), .b(new_n187), .o1(new_n188));
  inv000aa1d42x5               g093(.a(new_n187), .o1(new_n189));
  aoai13aa1n02x5               g094(.a(new_n189), .b(new_n181), .c(new_n179), .d(new_n180), .o1(new_n190));
  aoi022aa1n03x5               g095(.a(new_n190), .b(new_n186), .c(new_n185), .d(new_n188), .o1(\s[16] ));
  nano23aa1n02x4               g096(.a(new_n137), .b(new_n144), .c(new_n145), .d(new_n138), .out0(new_n192));
  nanp03aa1n02x5               g097(.a(new_n192), .b(new_n130), .c(new_n134), .o1(new_n193));
  nano32aa1n03x7               g098(.a(new_n193), .b(new_n186), .c(new_n178), .d(new_n182), .out0(new_n194));
  aoai13aa1n06x5               g099(.a(new_n194), .b(new_n129), .c(new_n110), .d(new_n121), .o1(new_n195));
  norb02aa1n06x4               g100(.a(new_n186), .b(new_n181), .out0(new_n196));
  aoai13aa1n12x5               g101(.a(new_n196), .b(new_n184), .c(new_n159), .d(new_n178), .o1(new_n197));
  oao003aa1n06x5               g102(.a(\a[16] ), .b(\b[15] ), .c(new_n189), .carry(new_n198));
  nanp03aa1d12x5               g103(.a(new_n195), .b(new_n197), .c(new_n198), .o1(new_n199));
  xorc02aa1n03x5               g104(.a(\a[17] ), .b(\b[16] ), .out0(new_n200));
  nano22aa1n02x4               g105(.a(new_n200), .b(new_n197), .c(new_n198), .out0(new_n201));
  aoi022aa1n02x5               g106(.a(new_n201), .b(new_n195), .c(new_n199), .d(new_n200), .o1(\s[17] ));
  nor042aa1d18x5               g107(.a(\b[16] ), .b(\a[17] ), .o1(new_n203));
  inv000aa1d42x5               g108(.a(new_n203), .o1(new_n204));
  nona23aa1n02x5               g109(.a(new_n175), .b(new_n162), .c(new_n161), .d(new_n174), .out0(new_n205));
  inv000aa1n02x5               g110(.a(new_n186), .o1(new_n206));
  nona32aa1n03x5               g111(.a(new_n152), .b(new_n206), .c(new_n181), .d(new_n205), .out0(new_n207));
  aoi012aa1d18x5               g112(.a(new_n207), .b(new_n169), .c(new_n171), .o1(new_n208));
  inv030aa1n02x5               g113(.a(new_n196), .o1(new_n209));
  aoai13aa1n06x5               g114(.a(new_n178), .b(new_n157), .c(new_n155), .d(new_n154), .o1(new_n210));
  aoai13aa1n09x5               g115(.a(new_n198), .b(new_n209), .c(new_n210), .d(new_n180), .o1(new_n211));
  oaih12aa1n02x5               g116(.a(new_n200), .b(new_n211), .c(new_n208), .o1(new_n212));
  nor042aa1d18x5               g117(.a(\b[17] ), .b(\a[18] ), .o1(new_n213));
  nand02aa1d12x5               g118(.a(\b[17] ), .b(\a[18] ), .o1(new_n214));
  norb02aa1n06x4               g119(.a(new_n214), .b(new_n213), .out0(new_n215));
  xnbna2aa1n03x5               g120(.a(new_n215), .b(new_n212), .c(new_n204), .out0(\s[18] ));
  and002aa1n02x5               g121(.a(new_n200), .b(new_n215), .o(new_n217));
  tech160nm_fioai012aa1n05x5   g122(.a(new_n217), .b(new_n211), .c(new_n208), .o1(new_n218));
  oaoi03aa1n02x5               g123(.a(\a[18] ), .b(\b[17] ), .c(new_n204), .o1(new_n219));
  inv000aa1d42x5               g124(.a(new_n219), .o1(new_n220));
  nor002aa1d32x5               g125(.a(\b[18] ), .b(\a[19] ), .o1(new_n221));
  nand02aa1d06x5               g126(.a(\b[18] ), .b(\a[19] ), .o1(new_n222));
  norb02aa1n12x5               g127(.a(new_n222), .b(new_n221), .out0(new_n223));
  xnbna2aa1n03x5               g128(.a(new_n223), .b(new_n218), .c(new_n220), .out0(\s[19] ));
  xnrc02aa1n02x5               g129(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  aoai13aa1n03x5               g130(.a(new_n223), .b(new_n219), .c(new_n199), .d(new_n217), .o1(new_n226));
  nor002aa1n12x5               g131(.a(\b[19] ), .b(\a[20] ), .o1(new_n227));
  nand02aa1d28x5               g132(.a(\b[19] ), .b(\a[20] ), .o1(new_n228));
  norb02aa1n02x5               g133(.a(new_n228), .b(new_n227), .out0(new_n229));
  inv000aa1d42x5               g134(.a(\a[19] ), .o1(new_n230));
  inv000aa1d42x5               g135(.a(\b[18] ), .o1(new_n231));
  aboi22aa1n03x5               g136(.a(new_n227), .b(new_n228), .c(new_n230), .d(new_n231), .out0(new_n232));
  inv000aa1d42x5               g137(.a(new_n221), .o1(new_n233));
  inv000aa1d42x5               g138(.a(new_n223), .o1(new_n234));
  aoai13aa1n02x5               g139(.a(new_n233), .b(new_n234), .c(new_n218), .d(new_n220), .o1(new_n235));
  aoi022aa1n03x5               g140(.a(new_n235), .b(new_n229), .c(new_n226), .d(new_n232), .o1(\s[20] ));
  nano32aa1n03x7               g141(.a(new_n234), .b(new_n200), .c(new_n229), .d(new_n215), .out0(new_n237));
  oaih12aa1n02x5               g142(.a(new_n237), .b(new_n211), .c(new_n208), .o1(new_n238));
  nanb03aa1n12x5               g143(.a(new_n227), .b(new_n228), .c(new_n222), .out0(new_n239));
  oai112aa1n06x5               g144(.a(new_n233), .b(new_n214), .c(new_n213), .d(new_n203), .o1(new_n240));
  aoi012aa1n12x5               g145(.a(new_n227), .b(new_n221), .c(new_n228), .o1(new_n241));
  oaih12aa1n12x5               g146(.a(new_n241), .b(new_n240), .c(new_n239), .o1(new_n242));
  nor042aa1d18x5               g147(.a(\b[20] ), .b(\a[21] ), .o1(new_n243));
  nand42aa1n06x5               g148(.a(\b[20] ), .b(\a[21] ), .o1(new_n244));
  norb02aa1n15x5               g149(.a(new_n244), .b(new_n243), .out0(new_n245));
  aoai13aa1n06x5               g150(.a(new_n245), .b(new_n242), .c(new_n199), .d(new_n237), .o1(new_n246));
  nano22aa1n03x7               g151(.a(new_n227), .b(new_n222), .c(new_n228), .out0(new_n247));
  oaih12aa1n02x5               g152(.a(new_n214), .b(\b[18] ), .c(\a[19] ), .o1(new_n248));
  oab012aa1n03x5               g153(.a(new_n248), .b(new_n203), .c(new_n213), .out0(new_n249));
  inv020aa1n02x5               g154(.a(new_n241), .o1(new_n250));
  aoi112aa1n02x5               g155(.a(new_n250), .b(new_n245), .c(new_n249), .d(new_n247), .o1(new_n251));
  aobi12aa1n03x7               g156(.a(new_n246), .b(new_n251), .c(new_n238), .out0(\s[21] ));
  nor042aa1n06x5               g157(.a(\b[21] ), .b(\a[22] ), .o1(new_n253));
  nand02aa1d12x5               g158(.a(\b[21] ), .b(\a[22] ), .o1(new_n254));
  norb02aa1n02x5               g159(.a(new_n254), .b(new_n253), .out0(new_n255));
  aoib12aa1n02x5               g160(.a(new_n243), .b(new_n254), .c(new_n253), .out0(new_n256));
  inv000aa1d42x5               g161(.a(new_n242), .o1(new_n257));
  inv000aa1d42x5               g162(.a(new_n243), .o1(new_n258));
  inv000aa1d42x5               g163(.a(new_n245), .o1(new_n259));
  aoai13aa1n03x5               g164(.a(new_n258), .b(new_n259), .c(new_n238), .d(new_n257), .o1(new_n260));
  aoi022aa1n02x5               g165(.a(new_n260), .b(new_n255), .c(new_n246), .d(new_n256), .o1(\s[22] ));
  inv000aa1n02x5               g166(.a(new_n237), .o1(new_n262));
  nano22aa1n02x4               g167(.a(new_n262), .b(new_n245), .c(new_n255), .out0(new_n263));
  tech160nm_fioai012aa1n05x5   g168(.a(new_n263), .b(new_n211), .c(new_n208), .o1(new_n264));
  nano23aa1d15x5               g169(.a(new_n243), .b(new_n253), .c(new_n254), .d(new_n244), .out0(new_n265));
  aoi012aa1d24x5               g170(.a(new_n253), .b(new_n243), .c(new_n254), .o1(new_n266));
  inv000aa1d42x5               g171(.a(new_n266), .o1(new_n267));
  aoi012aa1n02x5               g172(.a(new_n267), .b(new_n242), .c(new_n265), .o1(new_n268));
  nanp02aa1n03x5               g173(.a(new_n264), .b(new_n268), .o1(new_n269));
  xorc02aa1n12x5               g174(.a(\a[23] ), .b(\b[22] ), .out0(new_n270));
  aoi112aa1n02x5               g175(.a(new_n270), .b(new_n267), .c(new_n242), .d(new_n265), .o1(new_n271));
  aoi022aa1n02x5               g176(.a(new_n269), .b(new_n270), .c(new_n264), .d(new_n271), .o1(\s[23] ));
  nand02aa1n02x5               g177(.a(new_n269), .b(new_n270), .o1(new_n273));
  xorc02aa1n03x5               g178(.a(\a[24] ), .b(\b[23] ), .out0(new_n274));
  nor042aa1n06x5               g179(.a(\b[22] ), .b(\a[23] ), .o1(new_n275));
  norp02aa1n02x5               g180(.a(new_n274), .b(new_n275), .o1(new_n276));
  inv000aa1d42x5               g181(.a(new_n275), .o1(new_n277));
  inv000aa1d42x5               g182(.a(new_n270), .o1(new_n278));
  aoai13aa1n02x7               g183(.a(new_n277), .b(new_n278), .c(new_n264), .d(new_n268), .o1(new_n279));
  aoi022aa1n03x5               g184(.a(new_n279), .b(new_n274), .c(new_n273), .d(new_n276), .o1(\s[24] ));
  and002aa1n12x5               g185(.a(new_n274), .b(new_n270), .o(new_n281));
  nano22aa1n02x4               g186(.a(new_n262), .b(new_n281), .c(new_n265), .out0(new_n282));
  oaih12aa1n02x5               g187(.a(new_n282), .b(new_n211), .c(new_n208), .o1(new_n283));
  aoai13aa1n04x5               g188(.a(new_n265), .b(new_n250), .c(new_n249), .d(new_n247), .o1(new_n284));
  inv000aa1n06x5               g189(.a(new_n281), .o1(new_n285));
  oao003aa1n02x5               g190(.a(\a[24] ), .b(\b[23] ), .c(new_n277), .carry(new_n286));
  aoai13aa1n12x5               g191(.a(new_n286), .b(new_n285), .c(new_n284), .d(new_n266), .o1(new_n287));
  xorc02aa1n12x5               g192(.a(\a[25] ), .b(\b[24] ), .out0(new_n288));
  aoai13aa1n06x5               g193(.a(new_n288), .b(new_n287), .c(new_n199), .d(new_n282), .o1(new_n289));
  aoai13aa1n04x5               g194(.a(new_n281), .b(new_n267), .c(new_n242), .d(new_n265), .o1(new_n290));
  inv000aa1d42x5               g195(.a(new_n288), .o1(new_n291));
  and003aa1n02x5               g196(.a(new_n290), .b(new_n291), .c(new_n286), .o(new_n292));
  aobi12aa1n03x7               g197(.a(new_n289), .b(new_n292), .c(new_n283), .out0(\s[25] ));
  xorc02aa1n03x5               g198(.a(\a[26] ), .b(\b[25] ), .out0(new_n294));
  nor042aa1n03x5               g199(.a(\b[24] ), .b(\a[25] ), .o1(new_n295));
  norp02aa1n02x5               g200(.a(new_n294), .b(new_n295), .o1(new_n296));
  inv000aa1d42x5               g201(.a(new_n287), .o1(new_n297));
  inv000aa1d42x5               g202(.a(new_n295), .o1(new_n298));
  aoai13aa1n03x5               g203(.a(new_n298), .b(new_n291), .c(new_n283), .d(new_n297), .o1(new_n299));
  aoi022aa1n02x5               g204(.a(new_n299), .b(new_n294), .c(new_n289), .d(new_n296), .o1(\s[26] ));
  and002aa1n12x5               g205(.a(new_n294), .b(new_n288), .o(new_n301));
  nano32aa1n03x7               g206(.a(new_n262), .b(new_n301), .c(new_n265), .d(new_n281), .out0(new_n302));
  oai012aa1n18x5               g207(.a(new_n302), .b(new_n211), .c(new_n208), .o1(new_n303));
  inv000aa1d42x5               g208(.a(new_n301), .o1(new_n304));
  oao003aa1n02x5               g209(.a(\a[26] ), .b(\b[25] ), .c(new_n298), .carry(new_n305));
  aoai13aa1n04x5               g210(.a(new_n305), .b(new_n304), .c(new_n290), .d(new_n286), .o1(new_n306));
  xorc02aa1n12x5               g211(.a(\a[27] ), .b(\b[26] ), .out0(new_n307));
  aoai13aa1n06x5               g212(.a(new_n307), .b(new_n306), .c(new_n199), .d(new_n302), .o1(new_n308));
  inv000aa1d42x5               g213(.a(new_n305), .o1(new_n309));
  aoi112aa1n02x5               g214(.a(new_n307), .b(new_n309), .c(new_n287), .d(new_n301), .o1(new_n310));
  aobi12aa1n02x7               g215(.a(new_n308), .b(new_n310), .c(new_n303), .out0(\s[27] ));
  xorc02aa1n02x5               g216(.a(\a[28] ), .b(\b[27] ), .out0(new_n312));
  norp02aa1n02x5               g217(.a(\b[26] ), .b(\a[27] ), .o1(new_n313));
  norp02aa1n02x5               g218(.a(new_n312), .b(new_n313), .o1(new_n314));
  aoi012aa1d18x5               g219(.a(new_n309), .b(new_n287), .c(new_n301), .o1(new_n315));
  inv000aa1n03x5               g220(.a(new_n313), .o1(new_n316));
  inv000aa1d42x5               g221(.a(new_n307), .o1(new_n317));
  aoai13aa1n06x5               g222(.a(new_n316), .b(new_n317), .c(new_n303), .d(new_n315), .o1(new_n318));
  aoi022aa1n03x5               g223(.a(new_n318), .b(new_n312), .c(new_n308), .d(new_n314), .o1(\s[28] ));
  and002aa1n02x5               g224(.a(new_n312), .b(new_n307), .o(new_n320));
  aoai13aa1n02x5               g225(.a(new_n320), .b(new_n306), .c(new_n199), .d(new_n302), .o1(new_n321));
  inv000aa1d42x5               g226(.a(new_n320), .o1(new_n322));
  oao003aa1n02x5               g227(.a(\a[28] ), .b(\b[27] ), .c(new_n316), .carry(new_n323));
  aoai13aa1n06x5               g228(.a(new_n323), .b(new_n322), .c(new_n303), .d(new_n315), .o1(new_n324));
  xorc02aa1n02x5               g229(.a(\a[29] ), .b(\b[28] ), .out0(new_n325));
  norb02aa1n02x5               g230(.a(new_n323), .b(new_n325), .out0(new_n326));
  aoi022aa1n03x5               g231(.a(new_n324), .b(new_n325), .c(new_n321), .d(new_n326), .o1(\s[29] ));
  xorb03aa1n02x5               g232(.a(new_n100), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n03x7               g233(.a(new_n317), .b(new_n312), .c(new_n325), .out0(new_n329));
  aoai13aa1n02x5               g234(.a(new_n329), .b(new_n306), .c(new_n199), .d(new_n302), .o1(new_n330));
  inv000aa1d42x5               g235(.a(new_n329), .o1(new_n331));
  inv000aa1d42x5               g236(.a(\b[28] ), .o1(new_n332));
  inv000aa1d42x5               g237(.a(\a[29] ), .o1(new_n333));
  oaib12aa1n02x5               g238(.a(new_n323), .b(\b[28] ), .c(new_n333), .out0(new_n334));
  oaib12aa1n02x5               g239(.a(new_n334), .b(new_n332), .c(\a[29] ), .out0(new_n335));
  aoai13aa1n06x5               g240(.a(new_n335), .b(new_n331), .c(new_n303), .d(new_n315), .o1(new_n336));
  xorc02aa1n02x5               g241(.a(\a[30] ), .b(\b[29] ), .out0(new_n337));
  oaoi13aa1n02x5               g242(.a(new_n337), .b(new_n334), .c(new_n333), .d(new_n332), .o1(new_n338));
  aoi022aa1n03x5               g243(.a(new_n336), .b(new_n337), .c(new_n330), .d(new_n338), .o1(\s[30] ));
  nanb02aa1n02x5               g244(.a(\b[30] ), .b(\a[31] ), .out0(new_n340));
  nanb02aa1n02x5               g245(.a(\a[31] ), .b(\b[30] ), .out0(new_n341));
  nanp02aa1n02x5               g246(.a(new_n341), .b(new_n340), .o1(new_n342));
  nano32aa1d15x5               g247(.a(new_n317), .b(new_n337), .c(new_n312), .d(new_n325), .out0(new_n343));
  aoai13aa1n02x5               g248(.a(new_n343), .b(new_n306), .c(new_n199), .d(new_n302), .o1(new_n344));
  inv000aa1d42x5               g249(.a(new_n343), .o1(new_n345));
  norp02aa1n02x5               g250(.a(\b[29] ), .b(\a[30] ), .o1(new_n346));
  aoi022aa1n02x5               g251(.a(\b[29] ), .b(\a[30] ), .c(\a[29] ), .d(\b[28] ), .o1(new_n347));
  aoi012aa1n02x5               g252(.a(new_n346), .b(new_n334), .c(new_n347), .o1(new_n348));
  aoai13aa1n06x5               g253(.a(new_n348), .b(new_n345), .c(new_n303), .d(new_n315), .o1(new_n349));
  oai112aa1n02x5               g254(.a(new_n340), .b(new_n341), .c(\b[29] ), .d(\a[30] ), .o1(new_n350));
  aoi012aa1n02x5               g255(.a(new_n350), .b(new_n334), .c(new_n347), .o1(new_n351));
  aoi022aa1n03x5               g256(.a(new_n349), .b(new_n342), .c(new_n344), .d(new_n351), .o1(\s[31] ));
  nanp03aa1n02x5               g257(.a(new_n99), .b(new_n100), .c(new_n101), .o1(new_n353));
  nanb02aa1n02x5               g258(.a(new_n105), .b(new_n106), .out0(new_n354));
  xobna2aa1n03x5               g259(.a(new_n354), .b(new_n353), .c(new_n99), .out0(\s[3] ));
  nanb02aa1n02x5               g260(.a(new_n354), .b(new_n102), .out0(new_n356));
  aoib12aa1n02x5               g261(.a(new_n105), .b(new_n104), .c(new_n103), .out0(new_n357));
  aboi22aa1n03x5               g262(.a(new_n103), .b(new_n110), .c(new_n356), .d(new_n357), .out0(\s[4] ));
  xorc02aa1n02x5               g263(.a(\a[5] ), .b(\b[4] ), .out0(new_n359));
  xnbna2aa1n03x5               g264(.a(new_n359), .b(new_n108), .c(new_n109), .out0(\s[5] ));
  oaoi03aa1n02x5               g265(.a(new_n118), .b(new_n119), .c(new_n110), .o1(new_n361));
  aoai13aa1n02x5               g266(.a(new_n359), .b(new_n168), .c(new_n107), .d(new_n102), .o1(new_n362));
  nanb02aa1n02x5               g267(.a(new_n126), .b(new_n362), .out0(new_n363));
  aoai13aa1n02x5               g268(.a(new_n363), .b(new_n361), .c(new_n124), .d(new_n125), .o1(\s[6] ));
  inv000aa1d42x5               g269(.a(new_n113), .o1(new_n365));
  nanp02aa1n02x5               g270(.a(\b[6] ), .b(\a[7] ), .o1(new_n366));
  aoi022aa1n02x5               g271(.a(new_n363), .b(new_n125), .c(new_n366), .d(new_n365), .o1(new_n367));
  aoi013aa1n02x4               g272(.a(new_n367), .b(new_n363), .c(new_n127), .d(new_n365), .o1(\s[7] ));
  norp02aa1n02x5               g273(.a(new_n122), .b(new_n112), .o1(new_n369));
  nanp03aa1n02x5               g274(.a(new_n363), .b(new_n365), .c(new_n127), .o1(new_n370));
  xnbna2aa1n03x5               g275(.a(new_n369), .b(new_n370), .c(new_n365), .out0(\s[8] ));
  aoi113aa1n02x5               g276(.a(new_n130), .b(new_n170), .c(new_n123), .d(new_n126), .e(new_n127), .o1(new_n372));
  aoi022aa1n02x5               g277(.a(new_n172), .b(new_n130), .c(new_n169), .d(new_n372), .o1(\s[9] ));
endmodule


