// Benchmark "adder" written by ABC on Wed Jul 17 15:38:46 2024

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
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n138, new_n139,
    new_n140, new_n141, new_n142, new_n143, new_n144, new_n145, new_n146,
    new_n148, new_n149, new_n150, new_n151, new_n152, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n161, new_n162,
    new_n163, new_n164, new_n166, new_n167, new_n168, new_n170, new_n171,
    new_n172, new_n173, new_n174, new_n176, new_n177, new_n178, new_n179,
    new_n180, new_n181, new_n182, new_n183, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n197, new_n199, new_n200, new_n201, new_n202,
    new_n204, new_n205, new_n206, new_n207, new_n208, new_n211, new_n212,
    new_n213, new_n214, new_n215, new_n216, new_n217, new_n218, new_n219,
    new_n220, new_n221, new_n223, new_n224, new_n225, new_n226, new_n227,
    new_n228, new_n229, new_n231, new_n232, new_n233, new_n234, new_n235,
    new_n236, new_n237, new_n238, new_n239, new_n241, new_n242, new_n243,
    new_n244, new_n245, new_n246, new_n247, new_n248, new_n250, new_n251,
    new_n252, new_n253, new_n254, new_n255, new_n256, new_n257, new_n258,
    new_n259, new_n260, new_n262, new_n263, new_n264, new_n265, new_n266,
    new_n267, new_n268, new_n270, new_n271, new_n272, new_n273, new_n274,
    new_n275, new_n276, new_n277, new_n278, new_n280, new_n281, new_n282,
    new_n283, new_n284, new_n285, new_n286, new_n287, new_n288, new_n289,
    new_n290, new_n292, new_n293, new_n294, new_n295, new_n296, new_n297,
    new_n298, new_n299, new_n300, new_n301, new_n303, new_n304, new_n305,
    new_n306, new_n307, new_n308, new_n309, new_n312, new_n313, new_n314,
    new_n315, new_n316, new_n317, new_n318, new_n320, new_n321, new_n322,
    new_n323, new_n324, new_n325, new_n326, new_n329, new_n332, new_n333,
    new_n335, new_n336, new_n337, new_n338, new_n339, new_n341, new_n342;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  inv000aa1d42x5               g001(.a(\a[9] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(\b[8] ), .o1(new_n98));
  nanp02aa1n02x5               g003(.a(new_n98), .b(new_n97), .o1(new_n99));
  inv000aa1d42x5               g004(.a(\a[1] ), .o1(new_n100));
  inv040aa1d32x5               g005(.a(\b[0] ), .o1(new_n101));
  nor002aa1n02x5               g006(.a(\b[1] ), .b(\a[2] ), .o1(new_n102));
  nanp02aa1n02x5               g007(.a(\b[1] ), .b(\a[2] ), .o1(new_n103));
  oaoi13aa1n09x5               g008(.a(new_n102), .b(new_n103), .c(new_n100), .d(new_n101), .o1(new_n104));
  nor022aa1n04x5               g009(.a(\b[3] ), .b(\a[4] ), .o1(new_n105));
  nand22aa1n04x5               g010(.a(\b[3] ), .b(\a[4] ), .o1(new_n106));
  nor042aa1n04x5               g011(.a(\b[2] ), .b(\a[3] ), .o1(new_n107));
  nanp02aa1n02x5               g012(.a(\b[2] ), .b(\a[3] ), .o1(new_n108));
  nona23aa1n03x5               g013(.a(new_n108), .b(new_n106), .c(new_n105), .d(new_n107), .out0(new_n109));
  oaih12aa1n02x5               g014(.a(new_n106), .b(new_n107), .c(new_n105), .o1(new_n110));
  oaih12aa1n06x5               g015(.a(new_n110), .b(new_n109), .c(new_n104), .o1(new_n111));
  nanp02aa1n02x5               g016(.a(\b[4] ), .b(\a[5] ), .o1(new_n112));
  nor002aa1d32x5               g017(.a(\b[4] ), .b(\a[5] ), .o1(new_n113));
  nor022aa1n16x5               g018(.a(\b[5] ), .b(\a[6] ), .o1(new_n114));
  nand02aa1n06x5               g019(.a(\b[5] ), .b(\a[6] ), .o1(new_n115));
  nona23aa1n02x4               g020(.a(new_n112), .b(new_n115), .c(new_n114), .d(new_n113), .out0(new_n116));
  nand02aa1n03x5               g021(.a(\b[6] ), .b(\a[7] ), .o1(new_n117));
  nor042aa1n04x5               g022(.a(\b[7] ), .b(\a[8] ), .o1(new_n118));
  nanp02aa1n04x5               g023(.a(\b[7] ), .b(\a[8] ), .o1(new_n119));
  nor002aa1n20x5               g024(.a(\b[6] ), .b(\a[7] ), .o1(new_n120));
  nona23aa1n09x5               g025(.a(new_n119), .b(new_n117), .c(new_n120), .d(new_n118), .out0(new_n121));
  nor042aa1n03x5               g026(.a(new_n121), .b(new_n116), .o1(new_n122));
  aoi012aa1d18x5               g027(.a(new_n114), .b(new_n113), .c(new_n115), .o1(new_n123));
  oai012aa1n02x5               g028(.a(new_n119), .b(new_n120), .c(new_n118), .o1(new_n124));
  oai012aa1n12x5               g029(.a(new_n124), .b(new_n121), .c(new_n123), .o1(new_n125));
  xorc02aa1n02x5               g030(.a(\a[9] ), .b(\b[8] ), .out0(new_n126));
  aoai13aa1n06x5               g031(.a(new_n126), .b(new_n125), .c(new_n111), .d(new_n122), .o1(new_n127));
  nor002aa1d24x5               g032(.a(\b[9] ), .b(\a[10] ), .o1(new_n128));
  nand02aa1d16x5               g033(.a(\b[9] ), .b(\a[10] ), .o1(new_n129));
  norb02aa1n12x5               g034(.a(new_n129), .b(new_n128), .out0(new_n130));
  xnbna2aa1n03x5               g035(.a(new_n130), .b(new_n127), .c(new_n99), .out0(\s[10] ));
  nor022aa1n16x5               g036(.a(\b[10] ), .b(\a[11] ), .o1(new_n132));
  nand42aa1n04x5               g037(.a(\b[10] ), .b(\a[11] ), .o1(new_n133));
  norb02aa1n02x5               g038(.a(new_n133), .b(new_n132), .out0(new_n134));
  inv000aa1d42x5               g039(.a(new_n128), .o1(new_n135));
  inv000aa1d42x5               g040(.a(new_n130), .o1(new_n136));
  aoai13aa1n04x5               g041(.a(new_n135), .b(new_n136), .c(new_n127), .d(new_n99), .o1(new_n137));
  and002aa1n02x5               g042(.a(\b[0] ), .b(\a[1] ), .o(new_n138));
  oaoi03aa1n02x5               g043(.a(\a[2] ), .b(\b[1] ), .c(new_n138), .o1(new_n139));
  nano23aa1n02x4               g044(.a(new_n105), .b(new_n107), .c(new_n108), .d(new_n106), .out0(new_n140));
  aobi12aa1n06x5               g045(.a(new_n110), .b(new_n140), .c(new_n139), .out0(new_n141));
  nano23aa1n02x4               g046(.a(new_n120), .b(new_n118), .c(new_n117), .d(new_n119), .out0(new_n142));
  nanb02aa1n03x5               g047(.a(new_n116), .b(new_n142), .out0(new_n143));
  oabi12aa1n18x5               g048(.a(new_n125), .b(new_n141), .c(new_n143), .out0(new_n144));
  oai022aa1n02x5               g049(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n145));
  aoai13aa1n02x5               g050(.a(new_n129), .b(new_n145), .c(new_n144), .d(new_n126), .o1(new_n146));
  mtn022aa1n02x5               g051(.a(new_n137), .b(new_n146), .sa(new_n134), .o1(\s[11] ));
  nor042aa1n06x5               g052(.a(\b[11] ), .b(\a[12] ), .o1(new_n148));
  nanp02aa1n04x5               g053(.a(\b[11] ), .b(\a[12] ), .o1(new_n149));
  norb02aa1n02x5               g054(.a(new_n149), .b(new_n148), .out0(new_n150));
  aoi112aa1n02x5               g055(.a(new_n150), .b(new_n132), .c(new_n137), .d(new_n133), .o1(new_n151));
  aoai13aa1n02x5               g056(.a(new_n150), .b(new_n132), .c(new_n137), .d(new_n133), .o1(new_n152));
  norb02aa1n02x7               g057(.a(new_n152), .b(new_n151), .out0(\s[12] ));
  nanp02aa1n02x5               g058(.a(\b[8] ), .b(\a[9] ), .o1(new_n154));
  nona23aa1n09x5               g059(.a(new_n149), .b(new_n133), .c(new_n132), .d(new_n148), .out0(new_n155));
  nano32aa1n02x4               g060(.a(new_n155), .b(new_n130), .c(new_n154), .d(new_n99), .out0(new_n156));
  aoai13aa1n06x5               g061(.a(new_n156), .b(new_n125), .c(new_n111), .d(new_n122), .o1(new_n157));
  aoai13aa1n04x5               g062(.a(new_n129), .b(new_n128), .c(new_n97), .d(new_n98), .o1(new_n158));
  tech160nm_fioai012aa1n03p5x5 g063(.a(new_n149), .b(new_n148), .c(new_n132), .o1(new_n159));
  oai012aa1d24x5               g064(.a(new_n159), .b(new_n155), .c(new_n158), .o1(new_n160));
  inv000aa1d42x5               g065(.a(new_n160), .o1(new_n161));
  nor002aa1d32x5               g066(.a(\b[12] ), .b(\a[13] ), .o1(new_n162));
  nand22aa1n03x5               g067(.a(\b[12] ), .b(\a[13] ), .o1(new_n163));
  norb02aa1n02x5               g068(.a(new_n163), .b(new_n162), .out0(new_n164));
  xnbna2aa1n03x5               g069(.a(new_n164), .b(new_n157), .c(new_n161), .out0(\s[13] ));
  inv000aa1d42x5               g070(.a(new_n162), .o1(new_n166));
  inv000aa1n02x5               g071(.a(new_n163), .o1(new_n167));
  aoai13aa1n02x5               g072(.a(new_n166), .b(new_n167), .c(new_n157), .d(new_n161), .o1(new_n168));
  xorb03aa1n02x5               g073(.a(new_n168), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  nor002aa1n04x5               g074(.a(\b[13] ), .b(\a[14] ), .o1(new_n170));
  nand42aa1n08x5               g075(.a(\b[13] ), .b(\a[14] ), .o1(new_n171));
  nona23aa1n03x5               g076(.a(new_n171), .b(new_n163), .c(new_n162), .d(new_n170), .out0(new_n172));
  tech160nm_fioai012aa1n04x5   g077(.a(new_n171), .b(new_n170), .c(new_n162), .o1(new_n173));
  aoai13aa1n06x5               g078(.a(new_n173), .b(new_n172), .c(new_n157), .d(new_n161), .o1(new_n174));
  xorb03aa1n02x5               g079(.a(new_n174), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nor022aa1n08x5               g080(.a(\b[14] ), .b(\a[15] ), .o1(new_n176));
  nand02aa1d04x5               g081(.a(\b[14] ), .b(\a[15] ), .o1(new_n177));
  nor002aa1n03x5               g082(.a(\b[15] ), .b(\a[16] ), .o1(new_n178));
  nanp02aa1n04x5               g083(.a(\b[15] ), .b(\a[16] ), .o1(new_n179));
  nanb02aa1n02x5               g084(.a(new_n178), .b(new_n179), .out0(new_n180));
  inv000aa1d42x5               g085(.a(new_n180), .o1(new_n181));
  aoi112aa1n03x5               g086(.a(new_n176), .b(new_n181), .c(new_n174), .d(new_n177), .o1(new_n182));
  aoai13aa1n04x5               g087(.a(new_n181), .b(new_n176), .c(new_n174), .d(new_n177), .o1(new_n183));
  norb02aa1n03x4               g088(.a(new_n183), .b(new_n182), .out0(\s[16] ));
  nano23aa1n02x5               g089(.a(new_n132), .b(new_n148), .c(new_n149), .d(new_n133), .out0(new_n185));
  norb03aa1n02x5               g090(.a(new_n171), .b(new_n162), .c(new_n170), .out0(new_n186));
  inv000aa1n02x5               g091(.a(new_n177), .o1(new_n187));
  norb03aa1n03x4               g092(.a(new_n179), .b(new_n176), .c(new_n178), .out0(new_n188));
  nona23aa1n02x4               g093(.a(new_n188), .b(new_n186), .c(new_n167), .d(new_n187), .out0(new_n189));
  nano32aa1n03x7               g094(.a(new_n189), .b(new_n185), .c(new_n130), .d(new_n126), .out0(new_n190));
  aoai13aa1n06x5               g095(.a(new_n190), .b(new_n125), .c(new_n111), .d(new_n122), .o1(new_n191));
  nona23aa1n03x5               g096(.a(new_n179), .b(new_n177), .c(new_n176), .d(new_n178), .out0(new_n192));
  nor042aa1n02x5               g097(.a(new_n192), .b(new_n172), .o1(new_n193));
  oa0012aa1n02x5               g098(.a(new_n179), .b(new_n178), .c(new_n176), .o(new_n194));
  tech160nm_finor002aa1n03p5x5 g099(.a(new_n192), .b(new_n173), .o1(new_n195));
  aoi112aa1n09x5               g100(.a(new_n195), .b(new_n194), .c(new_n160), .d(new_n193), .o1(new_n196));
  nand02aa1d06x5               g101(.a(new_n191), .b(new_n196), .o1(new_n197));
  xorb03aa1n02x5               g102(.a(new_n197), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv040aa1d32x5               g103(.a(\a[18] ), .o1(new_n199));
  inv040aa1d30x5               g104(.a(\a[17] ), .o1(new_n200));
  inv000aa1d42x5               g105(.a(\b[16] ), .o1(new_n201));
  oaoi03aa1n03x5               g106(.a(new_n200), .b(new_n201), .c(new_n197), .o1(new_n202));
  xorb03aa1n02x5               g107(.a(new_n202), .b(\b[17] ), .c(new_n199), .out0(\s[18] ));
  xroi22aa1d06x4               g108(.a(new_n200), .b(\b[16] ), .c(new_n199), .d(\b[17] ), .out0(new_n204));
  inv000aa1d42x5               g109(.a(new_n204), .o1(new_n205));
  oai022aa1n02x5               g110(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n206));
  oaib12aa1n02x5               g111(.a(new_n206), .b(new_n199), .c(\b[17] ), .out0(new_n207));
  aoai13aa1n06x5               g112(.a(new_n207), .b(new_n205), .c(new_n191), .d(new_n196), .o1(new_n208));
  xorb03aa1n02x5               g113(.a(new_n208), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g114(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor042aa1n04x5               g115(.a(\b[18] ), .b(\a[19] ), .o1(new_n211));
  nand02aa1n03x5               g116(.a(\b[18] ), .b(\a[19] ), .o1(new_n212));
  nor042aa1n02x5               g117(.a(\b[19] ), .b(\a[20] ), .o1(new_n213));
  nanp02aa1n02x5               g118(.a(\b[19] ), .b(\a[20] ), .o1(new_n214));
  nanb02aa1n02x5               g119(.a(new_n213), .b(new_n214), .out0(new_n215));
  inv000aa1d42x5               g120(.a(new_n215), .o1(new_n216));
  aoi112aa1n02x5               g121(.a(new_n211), .b(new_n216), .c(new_n208), .d(new_n212), .o1(new_n217));
  inv000aa1n02x5               g122(.a(new_n211), .o1(new_n218));
  norb02aa1n02x5               g123(.a(new_n212), .b(new_n211), .out0(new_n219));
  nand02aa1n03x5               g124(.a(new_n208), .b(new_n219), .o1(new_n220));
  aoi012aa1n03x5               g125(.a(new_n215), .b(new_n220), .c(new_n218), .o1(new_n221));
  norp02aa1n03x5               g126(.a(new_n221), .b(new_n217), .o1(\s[20] ));
  nano23aa1n06x5               g127(.a(new_n211), .b(new_n213), .c(new_n214), .d(new_n212), .out0(new_n223));
  nand02aa1n02x5               g128(.a(new_n204), .b(new_n223), .o1(new_n224));
  nanp02aa1n02x5               g129(.a(new_n201), .b(new_n200), .o1(new_n225));
  oaoi03aa1n02x5               g130(.a(\a[18] ), .b(\b[17] ), .c(new_n225), .o1(new_n226));
  oaoi03aa1n02x5               g131(.a(\a[20] ), .b(\b[19] ), .c(new_n218), .o1(new_n227));
  tech160nm_fiaoi012aa1n03p5x5 g132(.a(new_n227), .b(new_n223), .c(new_n226), .o1(new_n228));
  aoai13aa1n04x5               g133(.a(new_n228), .b(new_n224), .c(new_n191), .d(new_n196), .o1(new_n229));
  xorb03aa1n02x5               g134(.a(new_n229), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  inv000aa1d42x5               g135(.a(\a[21] ), .o1(new_n231));
  nanb02aa1n18x5               g136(.a(\b[20] ), .b(new_n231), .out0(new_n232));
  inv000aa1d42x5               g137(.a(new_n232), .o1(new_n233));
  xorc02aa1n02x5               g138(.a(\a[21] ), .b(\b[20] ), .out0(new_n234));
  xorc02aa1n02x5               g139(.a(\a[22] ), .b(\b[21] ), .out0(new_n235));
  aoi112aa1n03x4               g140(.a(new_n233), .b(new_n235), .c(new_n229), .d(new_n234), .o1(new_n236));
  nanp02aa1n03x5               g141(.a(new_n229), .b(new_n234), .o1(new_n237));
  inv000aa1d42x5               g142(.a(new_n235), .o1(new_n238));
  aoi012aa1n06x5               g143(.a(new_n238), .b(new_n237), .c(new_n232), .o1(new_n239));
  nor042aa1n03x5               g144(.a(new_n239), .b(new_n236), .o1(\s[22] ));
  inv000aa1d42x5               g145(.a(\a[22] ), .o1(new_n241));
  xroi22aa1d04x5               g146(.a(new_n231), .b(\b[20] ), .c(new_n241), .d(\b[21] ), .out0(new_n242));
  nand23aa1n06x5               g147(.a(new_n242), .b(new_n204), .c(new_n223), .o1(new_n243));
  nona23aa1n02x4               g148(.a(new_n214), .b(new_n212), .c(new_n211), .d(new_n213), .out0(new_n244));
  oabi12aa1n03x5               g149(.a(new_n227), .b(new_n244), .c(new_n207), .out0(new_n245));
  oaoi03aa1n02x5               g150(.a(\a[22] ), .b(\b[21] ), .c(new_n232), .o1(new_n246));
  aoi012aa1n02x5               g151(.a(new_n246), .b(new_n245), .c(new_n242), .o1(new_n247));
  aoai13aa1n04x5               g152(.a(new_n247), .b(new_n243), .c(new_n191), .d(new_n196), .o1(new_n248));
  xorb03aa1n02x5               g153(.a(new_n248), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  nor002aa1n16x5               g154(.a(\b[22] ), .b(\a[23] ), .o1(new_n250));
  nand22aa1n03x5               g155(.a(\b[22] ), .b(\a[23] ), .o1(new_n251));
  nor042aa1n02x5               g156(.a(\b[23] ), .b(\a[24] ), .o1(new_n252));
  nand02aa1n03x5               g157(.a(\b[23] ), .b(\a[24] ), .o1(new_n253));
  norb02aa1n02x5               g158(.a(new_n253), .b(new_n252), .out0(new_n254));
  aoi112aa1n03x4               g159(.a(new_n250), .b(new_n254), .c(new_n248), .d(new_n251), .o1(new_n255));
  inv000aa1d42x5               g160(.a(new_n250), .o1(new_n256));
  norb02aa1n02x5               g161(.a(new_n251), .b(new_n250), .out0(new_n257));
  nand02aa1n02x5               g162(.a(new_n248), .b(new_n257), .o1(new_n258));
  inv000aa1d42x5               g163(.a(new_n254), .o1(new_n259));
  aoi012aa1n06x5               g164(.a(new_n259), .b(new_n258), .c(new_n256), .o1(new_n260));
  nor042aa1n03x5               g165(.a(new_n260), .b(new_n255), .o1(\s[24] ));
  nano23aa1n06x5               g166(.a(new_n250), .b(new_n252), .c(new_n253), .d(new_n251), .out0(new_n262));
  nanb03aa1n03x5               g167(.a(new_n224), .b(new_n262), .c(new_n242), .out0(new_n263));
  nona22aa1n02x4               g168(.a(new_n253), .b(new_n252), .c(new_n250), .out0(new_n264));
  aoi022aa1n03x5               g169(.a(new_n262), .b(new_n246), .c(new_n253), .d(new_n264), .o1(new_n265));
  inv020aa1n03x5               g170(.a(new_n265), .o1(new_n266));
  aoi013aa1n02x4               g171(.a(new_n266), .b(new_n245), .c(new_n242), .d(new_n262), .o1(new_n267));
  aoai13aa1n04x5               g172(.a(new_n267), .b(new_n263), .c(new_n191), .d(new_n196), .o1(new_n268));
  xorb03aa1n02x5               g173(.a(new_n268), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  nor022aa1n03x5               g174(.a(\b[24] ), .b(\a[25] ), .o1(new_n270));
  nanp02aa1n02x5               g175(.a(\b[24] ), .b(\a[25] ), .o1(new_n271));
  xorc02aa1n06x5               g176(.a(\a[26] ), .b(\b[25] ), .out0(new_n272));
  aoi112aa1n03x4               g177(.a(new_n270), .b(new_n272), .c(new_n268), .d(new_n271), .o1(new_n273));
  inv000aa1n02x5               g178(.a(new_n270), .o1(new_n274));
  norb02aa1n02x5               g179(.a(new_n271), .b(new_n270), .out0(new_n275));
  nanp02aa1n03x5               g180(.a(new_n268), .b(new_n275), .o1(new_n276));
  inv040aa1n03x5               g181(.a(new_n272), .o1(new_n277));
  tech160nm_fiaoi012aa1n02p5x5 g182(.a(new_n277), .b(new_n276), .c(new_n274), .o1(new_n278));
  nor002aa1n02x5               g183(.a(new_n278), .b(new_n273), .o1(\s[26] ));
  nanp02aa1n02x5               g184(.a(new_n160), .b(new_n193), .o1(new_n280));
  nona22aa1n02x4               g185(.a(new_n280), .b(new_n195), .c(new_n194), .out0(new_n281));
  nano22aa1n03x7               g186(.a(new_n277), .b(new_n274), .c(new_n271), .out0(new_n282));
  nano22aa1n03x7               g187(.a(new_n243), .b(new_n282), .c(new_n262), .out0(new_n283));
  aoai13aa1n06x5               g188(.a(new_n283), .b(new_n281), .c(new_n144), .d(new_n190), .o1(new_n284));
  nano22aa1n03x7               g189(.a(new_n228), .b(new_n242), .c(new_n262), .out0(new_n285));
  oaoi03aa1n02x5               g190(.a(\a[26] ), .b(\b[25] ), .c(new_n274), .o1(new_n286));
  oaoi13aa1n09x5               g191(.a(new_n286), .b(new_n282), .c(new_n285), .d(new_n266), .o1(new_n287));
  norp02aa1n02x5               g192(.a(\b[26] ), .b(\a[27] ), .o1(new_n288));
  nanp02aa1n02x5               g193(.a(\b[26] ), .b(\a[27] ), .o1(new_n289));
  norb02aa1n02x5               g194(.a(new_n289), .b(new_n288), .out0(new_n290));
  xnbna2aa1n03x5               g195(.a(new_n290), .b(new_n284), .c(new_n287), .out0(\s[27] ));
  inv000aa1n06x5               g196(.a(new_n288), .o1(new_n292));
  aobi12aa1n02x7               g197(.a(new_n290), .b(new_n284), .c(new_n287), .out0(new_n293));
  xnrc02aa1n02x5               g198(.a(\b[27] ), .b(\a[28] ), .out0(new_n294));
  nano22aa1n03x5               g199(.a(new_n293), .b(new_n292), .c(new_n294), .out0(new_n295));
  nanp03aa1n02x5               g200(.a(new_n245), .b(new_n242), .c(new_n262), .o1(new_n296));
  inv000aa1d42x5               g201(.a(new_n282), .o1(new_n297));
  inv000aa1n02x5               g202(.a(new_n286), .o1(new_n298));
  aoai13aa1n06x5               g203(.a(new_n298), .b(new_n297), .c(new_n296), .d(new_n265), .o1(new_n299));
  aoai13aa1n03x5               g204(.a(new_n290), .b(new_n299), .c(new_n197), .d(new_n283), .o1(new_n300));
  tech160nm_fiaoi012aa1n02p5x5 g205(.a(new_n294), .b(new_n300), .c(new_n292), .o1(new_n301));
  norp02aa1n03x5               g206(.a(new_n301), .b(new_n295), .o1(\s[28] ));
  nano22aa1n02x4               g207(.a(new_n294), .b(new_n292), .c(new_n289), .out0(new_n303));
  aobi12aa1n02x7               g208(.a(new_n303), .b(new_n284), .c(new_n287), .out0(new_n304));
  oao003aa1n02x5               g209(.a(\a[28] ), .b(\b[27] ), .c(new_n292), .carry(new_n305));
  xnrc02aa1n02x5               g210(.a(\b[28] ), .b(\a[29] ), .out0(new_n306));
  nano22aa1n03x5               g211(.a(new_n304), .b(new_n305), .c(new_n306), .out0(new_n307));
  aoai13aa1n03x5               g212(.a(new_n303), .b(new_n299), .c(new_n197), .d(new_n283), .o1(new_n308));
  tech160nm_fiaoi012aa1n02p5x5 g213(.a(new_n306), .b(new_n308), .c(new_n305), .o1(new_n309));
  norp02aa1n03x5               g214(.a(new_n309), .b(new_n307), .o1(\s[29] ));
  xnrb03aa1n02x5               g215(.a(new_n138), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano23aa1n02x4               g216(.a(new_n306), .b(new_n294), .c(new_n289), .d(new_n292), .out0(new_n312));
  aoai13aa1n03x5               g217(.a(new_n312), .b(new_n299), .c(new_n197), .d(new_n283), .o1(new_n313));
  oao003aa1n02x5               g218(.a(\a[29] ), .b(\b[28] ), .c(new_n305), .carry(new_n314));
  xnrc02aa1n02x5               g219(.a(\b[29] ), .b(\a[30] ), .out0(new_n315));
  tech160nm_fiaoi012aa1n02p5x5 g220(.a(new_n315), .b(new_n313), .c(new_n314), .o1(new_n316));
  aobi12aa1n02x7               g221(.a(new_n312), .b(new_n284), .c(new_n287), .out0(new_n317));
  nano22aa1n03x5               g222(.a(new_n317), .b(new_n314), .c(new_n315), .out0(new_n318));
  norp02aa1n03x5               g223(.a(new_n316), .b(new_n318), .o1(\s[30] ));
  norb03aa1n02x5               g224(.a(new_n303), .b(new_n315), .c(new_n306), .out0(new_n320));
  aobi12aa1n02x7               g225(.a(new_n320), .b(new_n284), .c(new_n287), .out0(new_n321));
  oao003aa1n02x5               g226(.a(\a[30] ), .b(\b[29] ), .c(new_n314), .carry(new_n322));
  xnrc02aa1n02x5               g227(.a(\b[30] ), .b(\a[31] ), .out0(new_n323));
  nano22aa1n03x5               g228(.a(new_n321), .b(new_n322), .c(new_n323), .out0(new_n324));
  aoai13aa1n03x5               g229(.a(new_n320), .b(new_n299), .c(new_n197), .d(new_n283), .o1(new_n325));
  tech160nm_fiaoi012aa1n02p5x5 g230(.a(new_n323), .b(new_n325), .c(new_n322), .o1(new_n326));
  norp02aa1n03x5               g231(.a(new_n326), .b(new_n324), .o1(\s[31] ));
  xnrb03aa1n02x5               g232(.a(new_n104), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g233(.a(\a[3] ), .b(\b[2] ), .c(new_n104), .o1(new_n329));
  xorb03aa1n02x5               g234(.a(new_n329), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g235(.a(new_n111), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  norb02aa1n02x5               g236(.a(new_n115), .b(new_n114), .out0(new_n332));
  aoi012aa1n02x5               g237(.a(new_n113), .b(new_n111), .c(new_n112), .o1(new_n333));
  xnrc02aa1n02x5               g238(.a(new_n333), .b(new_n332), .out0(\s[6] ));
  inv000aa1d42x5               g239(.a(new_n123), .o1(new_n335));
  norb03aa1n03x5               g240(.a(new_n112), .b(new_n141), .c(new_n113), .out0(new_n336));
  norb02aa1n02x5               g241(.a(new_n117), .b(new_n120), .out0(new_n337));
  aoai13aa1n03x5               g242(.a(new_n337), .b(new_n335), .c(new_n336), .d(new_n332), .o1(new_n338));
  aoi112aa1n02x5               g243(.a(new_n337), .b(new_n335), .c(new_n336), .d(new_n332), .o1(new_n339));
  norb02aa1n02x5               g244(.a(new_n338), .b(new_n339), .out0(\s[7] ));
  norb02aa1n02x5               g245(.a(new_n119), .b(new_n118), .out0(new_n341));
  inv000aa1d42x5               g246(.a(new_n120), .o1(new_n342));
  xnbna2aa1n03x5               g247(.a(new_n341), .b(new_n338), .c(new_n342), .out0(\s[8] ));
  xorb03aa1n02x5               g248(.a(new_n144), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


