// Benchmark "adder" written by ABC on Thu Jul 18 06:01:36 2024

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
    new_n133, new_n134, new_n135, new_n137, new_n138, new_n139, new_n140,
    new_n141, new_n142, new_n143, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n159, new_n160, new_n161, new_n162, new_n163,
    new_n164, new_n165, new_n166, new_n167, new_n168, new_n169, new_n170,
    new_n171, new_n172, new_n174, new_n175, new_n176, new_n178, new_n179,
    new_n180, new_n181, new_n182, new_n183, new_n184, new_n185, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n197, new_n198, new_n199, new_n200, new_n202, new_n203,
    new_n204, new_n205, new_n206, new_n207, new_n208, new_n211, new_n212,
    new_n213, new_n214, new_n215, new_n216, new_n217, new_n218, new_n219,
    new_n220, new_n221, new_n223, new_n224, new_n225, new_n226, new_n227,
    new_n228, new_n229, new_n230, new_n231, new_n233, new_n234, new_n235,
    new_n236, new_n237, new_n238, new_n239, new_n240, new_n241, new_n243,
    new_n244, new_n245, new_n246, new_n247, new_n249, new_n250, new_n251,
    new_n252, new_n253, new_n254, new_n255, new_n256, new_n257, new_n258,
    new_n259, new_n261, new_n262, new_n263, new_n264, new_n265, new_n266,
    new_n267, new_n268, new_n270, new_n271, new_n272, new_n273, new_n274,
    new_n275, new_n276, new_n277, new_n279, new_n280, new_n281, new_n282,
    new_n283, new_n284, new_n285, new_n286, new_n287, new_n288, new_n289,
    new_n290, new_n291, new_n293, new_n294, new_n295, new_n296, new_n297,
    new_n298, new_n299, new_n300, new_n301, new_n302, new_n303, new_n305,
    new_n306, new_n307, new_n308, new_n309, new_n310, new_n311, new_n312,
    new_n315, new_n316, new_n317, new_n318, new_n319, new_n320, new_n321,
    new_n322, new_n324, new_n325, new_n326, new_n327, new_n328, new_n329,
    new_n330, new_n331, new_n334, new_n337, new_n339, new_n341;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nanp02aa1n02x5               g001(.a(\b[1] ), .b(\a[2] ), .o1(new_n97));
  nand02aa1d04x5               g002(.a(\b[0] ), .b(\a[1] ), .o1(new_n98));
  nor002aa1n03x5               g003(.a(\b[1] ), .b(\a[2] ), .o1(new_n99));
  oai012aa1n12x5               g004(.a(new_n97), .b(new_n99), .c(new_n98), .o1(new_n100));
  tech160nm_fixnrc02aa1n02p5x5 g005(.a(\b[3] ), .b(\a[4] ), .out0(new_n101));
  nor042aa1n04x5               g006(.a(\b[2] ), .b(\a[3] ), .o1(new_n102));
  nand02aa1n03x5               g007(.a(\b[2] ), .b(\a[3] ), .o1(new_n103));
  nanb02aa1n02x5               g008(.a(new_n102), .b(new_n103), .out0(new_n104));
  aoi112aa1n03x4               g009(.a(\b[2] ), .b(\a[3] ), .c(\a[4] ), .d(\b[3] ), .o1(new_n105));
  oab012aa1n04x5               g010(.a(new_n105), .b(\a[4] ), .c(\b[3] ), .out0(new_n106));
  oai013aa1n06x5               g011(.a(new_n106), .b(new_n101), .c(new_n100), .d(new_n104), .o1(new_n107));
  nor002aa1n06x5               g012(.a(\b[7] ), .b(\a[8] ), .o1(new_n108));
  nand22aa1n09x5               g013(.a(\b[7] ), .b(\a[8] ), .o1(new_n109));
  nanp02aa1n04x5               g014(.a(\b[6] ), .b(\a[7] ), .o1(new_n110));
  nor002aa1n04x5               g015(.a(\b[6] ), .b(\a[7] ), .o1(new_n111));
  nona23aa1n03x5               g016(.a(new_n110), .b(new_n109), .c(new_n111), .d(new_n108), .out0(new_n112));
  nor042aa1n04x5               g017(.a(\b[4] ), .b(\a[5] ), .o1(new_n113));
  tech160nm_finand02aa1n03p5x5 g018(.a(\b[4] ), .b(\a[5] ), .o1(new_n114));
  norb02aa1n03x4               g019(.a(new_n114), .b(new_n113), .out0(new_n115));
  inv000aa1n02x5               g020(.a(new_n115), .o1(new_n116));
  xnrc02aa1n02x5               g021(.a(\b[5] ), .b(\a[6] ), .out0(new_n117));
  nor043aa1n03x5               g022(.a(new_n112), .b(new_n116), .c(new_n117), .o1(new_n118));
  inv000aa1d42x5               g023(.a(\a[6] ), .o1(new_n119));
  inv000aa1d42x5               g024(.a(\b[5] ), .o1(new_n120));
  oaoi03aa1n02x5               g025(.a(new_n119), .b(new_n120), .c(new_n113), .o1(new_n121));
  ao0012aa1n03x5               g026(.a(new_n108), .b(new_n111), .c(new_n109), .o(new_n122));
  oabi12aa1n03x5               g027(.a(new_n122), .b(new_n112), .c(new_n121), .out0(new_n123));
  nor002aa1n20x5               g028(.a(\b[8] ), .b(\a[9] ), .o1(new_n124));
  nand02aa1n03x5               g029(.a(\b[8] ), .b(\a[9] ), .o1(new_n125));
  norb02aa1n02x5               g030(.a(new_n125), .b(new_n124), .out0(new_n126));
  aoai13aa1n02x5               g031(.a(new_n126), .b(new_n123), .c(new_n107), .d(new_n118), .o1(new_n127));
  oai012aa1n02x5               g032(.a(new_n127), .b(\b[8] ), .c(\a[9] ), .o1(new_n128));
  xorb03aa1n02x5               g033(.a(new_n128), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  nand02aa1n10x5               g034(.a(\b[9] ), .b(\a[10] ), .o1(new_n130));
  nor002aa1d32x5               g035(.a(\b[10] ), .b(\a[11] ), .o1(new_n131));
  nand02aa1n16x5               g036(.a(\b[10] ), .b(\a[11] ), .o1(new_n132));
  norb02aa1n02x5               g037(.a(new_n132), .b(new_n131), .out0(new_n133));
  nor002aa1n16x5               g038(.a(\b[9] ), .b(\a[10] ), .o1(new_n134));
  nona22aa1n02x4               g039(.a(new_n127), .b(new_n134), .c(new_n124), .out0(new_n135));
  xobna2aa1n03x5               g040(.a(new_n133), .b(new_n135), .c(new_n130), .out0(\s[11] ));
  nor002aa1d24x5               g041(.a(\b[11] ), .b(\a[12] ), .o1(new_n137));
  nand02aa1d24x5               g042(.a(\b[11] ), .b(\a[12] ), .o1(new_n138));
  norb02aa1n02x5               g043(.a(new_n138), .b(new_n137), .out0(new_n139));
  aoi113aa1n02x5               g044(.a(new_n131), .b(new_n139), .c(new_n135), .d(new_n132), .e(new_n130), .o1(new_n140));
  inv000aa1d42x5               g045(.a(new_n131), .o1(new_n141));
  nanp03aa1n02x5               g046(.a(new_n135), .b(new_n130), .c(new_n133), .o1(new_n142));
  aobi12aa1n02x5               g047(.a(new_n139), .b(new_n142), .c(new_n141), .out0(new_n143));
  norp02aa1n02x5               g048(.a(new_n143), .b(new_n140), .o1(\s[12] ));
  nano23aa1n09x5               g049(.a(new_n131), .b(new_n137), .c(new_n138), .d(new_n132), .out0(new_n145));
  nano23aa1n09x5               g050(.a(new_n124), .b(new_n134), .c(new_n130), .d(new_n125), .out0(new_n146));
  nand22aa1n09x5               g051(.a(new_n146), .b(new_n145), .o1(new_n147));
  inv000aa1d42x5               g052(.a(new_n147), .o1(new_n148));
  aoai13aa1n02x5               g053(.a(new_n148), .b(new_n123), .c(new_n107), .d(new_n118), .o1(new_n149));
  nona23aa1d18x5               g054(.a(new_n138), .b(new_n132), .c(new_n131), .d(new_n137), .out0(new_n150));
  oai012aa1d24x5               g055(.a(new_n130), .b(new_n134), .c(new_n124), .o1(new_n151));
  aoi012aa1d24x5               g056(.a(new_n137), .b(new_n131), .c(new_n138), .o1(new_n152));
  oai012aa1d24x5               g057(.a(new_n152), .b(new_n150), .c(new_n151), .o1(new_n153));
  inv000aa1d42x5               g058(.a(new_n153), .o1(new_n154));
  nor042aa1n02x5               g059(.a(\b[12] ), .b(\a[13] ), .o1(new_n155));
  nand42aa1n06x5               g060(.a(\b[12] ), .b(\a[13] ), .o1(new_n156));
  norb02aa1n03x5               g061(.a(new_n156), .b(new_n155), .out0(new_n157));
  xnbna2aa1n03x5               g062(.a(new_n157), .b(new_n149), .c(new_n154), .out0(\s[13] ));
  orn002aa1n02x5               g063(.a(\a[13] ), .b(\b[12] ), .o(new_n159));
  xorc02aa1n02x5               g064(.a(\a[4] ), .b(\b[3] ), .out0(new_n160));
  norb02aa1n02x5               g065(.a(new_n103), .b(new_n102), .out0(new_n161));
  nanb03aa1n02x5               g066(.a(new_n100), .b(new_n160), .c(new_n161), .out0(new_n162));
  nano23aa1n03x7               g067(.a(new_n111), .b(new_n108), .c(new_n109), .d(new_n110), .out0(new_n163));
  xorc02aa1n02x5               g068(.a(\a[6] ), .b(\b[5] ), .out0(new_n164));
  nanp03aa1n02x5               g069(.a(new_n163), .b(new_n115), .c(new_n164), .o1(new_n165));
  oao003aa1n02x5               g070(.a(new_n119), .b(new_n120), .c(new_n113), .carry(new_n166));
  tech160nm_fiaoi012aa1n03p5x5 g071(.a(new_n122), .b(new_n163), .c(new_n166), .o1(new_n167));
  aoai13aa1n04x5               g072(.a(new_n167), .b(new_n165), .c(new_n162), .d(new_n106), .o1(new_n168));
  aoai13aa1n02x5               g073(.a(new_n157), .b(new_n153), .c(new_n168), .d(new_n148), .o1(new_n169));
  nor042aa1n02x5               g074(.a(\b[13] ), .b(\a[14] ), .o1(new_n170));
  nand22aa1n02x5               g075(.a(\b[13] ), .b(\a[14] ), .o1(new_n171));
  norb02aa1n03x5               g076(.a(new_n171), .b(new_n170), .out0(new_n172));
  xnbna2aa1n03x5               g077(.a(new_n172), .b(new_n169), .c(new_n159), .out0(\s[14] ));
  nona23aa1n03x5               g078(.a(new_n171), .b(new_n156), .c(new_n155), .d(new_n170), .out0(new_n174));
  aoi012aa1n03x5               g079(.a(new_n170), .b(new_n155), .c(new_n171), .o1(new_n175));
  aoai13aa1n06x5               g080(.a(new_n175), .b(new_n174), .c(new_n149), .d(new_n154), .o1(new_n176));
  xorb03aa1n02x5               g081(.a(new_n176), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nor042aa1n02x5               g082(.a(\b[14] ), .b(\a[15] ), .o1(new_n178));
  tech160nm_finand02aa1n03p5x5 g083(.a(\b[14] ), .b(\a[15] ), .o1(new_n179));
  nor042aa1n02x5               g084(.a(\b[15] ), .b(\a[16] ), .o1(new_n180));
  nand42aa1n03x5               g085(.a(\b[15] ), .b(\a[16] ), .o1(new_n181));
  nanb02aa1n02x5               g086(.a(new_n180), .b(new_n181), .out0(new_n182));
  inv000aa1d42x5               g087(.a(new_n182), .o1(new_n183));
  aoi112aa1n02x5               g088(.a(new_n183), .b(new_n178), .c(new_n176), .d(new_n179), .o1(new_n184));
  aoai13aa1n02x5               g089(.a(new_n183), .b(new_n178), .c(new_n176), .d(new_n179), .o1(new_n185));
  norb02aa1n02x5               g090(.a(new_n185), .b(new_n184), .out0(\s[16] ));
  nano23aa1n02x5               g091(.a(new_n178), .b(new_n180), .c(new_n181), .d(new_n179), .out0(new_n187));
  nano32aa1n03x7               g092(.a(new_n147), .b(new_n187), .c(new_n157), .d(new_n172), .out0(new_n188));
  aoai13aa1n06x5               g093(.a(new_n188), .b(new_n123), .c(new_n107), .d(new_n118), .o1(new_n189));
  nona23aa1n09x5               g094(.a(new_n181), .b(new_n179), .c(new_n178), .d(new_n180), .out0(new_n190));
  norp02aa1n06x5               g095(.a(new_n190), .b(new_n174), .o1(new_n191));
  norp02aa1n03x5               g096(.a(new_n190), .b(new_n175), .o1(new_n192));
  oa0012aa1n02x5               g097(.a(new_n181), .b(new_n180), .c(new_n178), .o(new_n193));
  aoi112aa1n09x5               g098(.a(new_n193), .b(new_n192), .c(new_n153), .d(new_n191), .o1(new_n194));
  xorc02aa1n02x5               g099(.a(\a[17] ), .b(\b[16] ), .out0(new_n195));
  xnbna2aa1n03x5               g100(.a(new_n195), .b(new_n189), .c(new_n194), .out0(\s[17] ));
  inv040aa1d32x5               g101(.a(\a[18] ), .o1(new_n197));
  nanp02aa1n06x5               g102(.a(new_n189), .b(new_n194), .o1(new_n198));
  norp02aa1n02x5               g103(.a(\b[16] ), .b(\a[17] ), .o1(new_n199));
  tech160nm_fiaoi012aa1n05x5   g104(.a(new_n199), .b(new_n198), .c(new_n195), .o1(new_n200));
  xorb03aa1n02x5               g105(.a(new_n200), .b(\b[17] ), .c(new_n197), .out0(\s[18] ));
  inv000aa1d42x5               g106(.a(\a[17] ), .o1(new_n202));
  xroi22aa1d06x4               g107(.a(new_n202), .b(\b[16] ), .c(new_n197), .d(\b[17] ), .out0(new_n203));
  inv000aa1d42x5               g108(.a(new_n203), .o1(new_n204));
  inv000aa1d42x5               g109(.a(\b[17] ), .o1(new_n205));
  oao003aa1n02x5               g110(.a(new_n197), .b(new_n205), .c(new_n199), .carry(new_n206));
  inv000aa1d42x5               g111(.a(new_n206), .o1(new_n207));
  aoai13aa1n04x5               g112(.a(new_n207), .b(new_n204), .c(new_n189), .d(new_n194), .o1(new_n208));
  xorb03aa1n02x5               g113(.a(new_n208), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g114(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor042aa1n12x5               g115(.a(\b[18] ), .b(\a[19] ), .o1(new_n211));
  nanp02aa1n04x5               g116(.a(\b[18] ), .b(\a[19] ), .o1(new_n212));
  nor042aa1n12x5               g117(.a(\b[19] ), .b(\a[20] ), .o1(new_n213));
  nand02aa1d16x5               g118(.a(\b[19] ), .b(\a[20] ), .o1(new_n214));
  norb02aa1d21x5               g119(.a(new_n214), .b(new_n213), .out0(new_n215));
  aoi112aa1n03x4               g120(.a(new_n211), .b(new_n215), .c(new_n208), .d(new_n212), .o1(new_n216));
  inv000aa1d42x5               g121(.a(new_n211), .o1(new_n217));
  norb02aa1n06x4               g122(.a(new_n212), .b(new_n211), .out0(new_n218));
  nanp02aa1n03x5               g123(.a(new_n208), .b(new_n218), .o1(new_n219));
  inv000aa1d42x5               g124(.a(new_n215), .o1(new_n220));
  aoi012aa1n06x5               g125(.a(new_n220), .b(new_n219), .c(new_n217), .o1(new_n221));
  nor002aa1n02x5               g126(.a(new_n221), .b(new_n216), .o1(\s[20] ));
  nano23aa1d18x5               g127(.a(new_n211), .b(new_n213), .c(new_n214), .d(new_n212), .out0(new_n223));
  nand02aa1n02x5               g128(.a(new_n203), .b(new_n223), .o1(new_n224));
  aoi112aa1n02x5               g129(.a(\b[18] ), .b(\a[19] ), .c(\a[20] ), .d(\b[19] ), .o1(new_n225));
  norp02aa1n02x5               g130(.a(\b[17] ), .b(\a[18] ), .o1(new_n226));
  aoi112aa1n03x5               g131(.a(\b[16] ), .b(\a[17] ), .c(\a[18] ), .d(\b[17] ), .o1(new_n227));
  oai112aa1n06x5               g132(.a(new_n218), .b(new_n215), .c(new_n227), .d(new_n226), .o1(new_n228));
  nona22aa1d18x5               g133(.a(new_n228), .b(new_n225), .c(new_n213), .out0(new_n229));
  inv000aa1d42x5               g134(.a(new_n229), .o1(new_n230));
  aoai13aa1n06x5               g135(.a(new_n230), .b(new_n224), .c(new_n189), .d(new_n194), .o1(new_n231));
  xorb03aa1n02x5               g136(.a(new_n231), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor042aa1n03x5               g137(.a(\b[20] ), .b(\a[21] ), .o1(new_n233));
  xnrc02aa1n12x5               g138(.a(\b[20] ), .b(\a[21] ), .out0(new_n234));
  inv000aa1d42x5               g139(.a(new_n234), .o1(new_n235));
  xnrc02aa1n12x5               g140(.a(\b[21] ), .b(\a[22] ), .out0(new_n236));
  inv000aa1d42x5               g141(.a(new_n236), .o1(new_n237));
  aoi112aa1n03x4               g142(.a(new_n233), .b(new_n237), .c(new_n231), .d(new_n235), .o1(new_n238));
  inv000aa1n02x5               g143(.a(new_n233), .o1(new_n239));
  nanp02aa1n03x5               g144(.a(new_n231), .b(new_n235), .o1(new_n240));
  tech160nm_fiaoi012aa1n02p5x5 g145(.a(new_n236), .b(new_n240), .c(new_n239), .o1(new_n241));
  norp02aa1n03x5               g146(.a(new_n241), .b(new_n238), .o1(\s[22] ));
  nor042aa1n04x5               g147(.a(new_n236), .b(new_n234), .o1(new_n243));
  nand23aa1n06x5               g148(.a(new_n203), .b(new_n243), .c(new_n223), .o1(new_n244));
  oaoi03aa1n02x5               g149(.a(\a[22] ), .b(\b[21] ), .c(new_n239), .o1(new_n245));
  aoi012aa1n02x5               g150(.a(new_n245), .b(new_n229), .c(new_n243), .o1(new_n246));
  aoai13aa1n04x5               g151(.a(new_n246), .b(new_n244), .c(new_n189), .d(new_n194), .o1(new_n247));
  xorb03aa1n02x5               g152(.a(new_n247), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  nor042aa1n09x5               g153(.a(\b[22] ), .b(\a[23] ), .o1(new_n249));
  nanp02aa1n02x5               g154(.a(\b[22] ), .b(\a[23] ), .o1(new_n250));
  norb02aa1n02x5               g155(.a(new_n250), .b(new_n249), .out0(new_n251));
  nor042aa1n03x5               g156(.a(\b[23] ), .b(\a[24] ), .o1(new_n252));
  nand22aa1n02x5               g157(.a(\b[23] ), .b(\a[24] ), .o1(new_n253));
  norb02aa1n06x5               g158(.a(new_n253), .b(new_n252), .out0(new_n254));
  aoi112aa1n03x4               g159(.a(new_n249), .b(new_n254), .c(new_n247), .d(new_n251), .o1(new_n255));
  inv000aa1d42x5               g160(.a(new_n249), .o1(new_n256));
  nanp02aa1n03x5               g161(.a(new_n247), .b(new_n251), .o1(new_n257));
  inv000aa1d42x5               g162(.a(new_n254), .o1(new_n258));
  tech160nm_fiaoi012aa1n05x5   g163(.a(new_n258), .b(new_n257), .c(new_n256), .o1(new_n259));
  nor002aa1n02x5               g164(.a(new_n259), .b(new_n255), .o1(\s[24] ));
  nona23aa1n03x5               g165(.a(new_n253), .b(new_n250), .c(new_n249), .d(new_n252), .out0(new_n261));
  inv000aa1n02x5               g166(.a(new_n261), .o1(new_n262));
  nanb03aa1n03x5               g167(.a(new_n224), .b(new_n262), .c(new_n243), .out0(new_n263));
  aoi112aa1n02x5               g168(.a(\b[22] ), .b(\a[23] ), .c(\a[24] ), .d(\b[23] ), .o1(new_n264));
  aoi113aa1n03x7               g169(.a(new_n264), .b(new_n252), .c(new_n245), .d(new_n254), .e(new_n251), .o1(new_n265));
  inv000aa1n02x5               g170(.a(new_n265), .o1(new_n266));
  aoi013aa1n06x4               g171(.a(new_n266), .b(new_n229), .c(new_n243), .d(new_n262), .o1(new_n267));
  aoai13aa1n04x5               g172(.a(new_n267), .b(new_n263), .c(new_n189), .d(new_n194), .o1(new_n268));
  xorb03aa1n02x5               g173(.a(new_n268), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g174(.a(\b[24] ), .b(\a[25] ), .o1(new_n270));
  xorc02aa1n06x5               g175(.a(\a[25] ), .b(\b[24] ), .out0(new_n271));
  xorc02aa1n12x5               g176(.a(\a[26] ), .b(\b[25] ), .out0(new_n272));
  aoi112aa1n03x4               g177(.a(new_n270), .b(new_n272), .c(new_n268), .d(new_n271), .o1(new_n273));
  inv000aa1d42x5               g178(.a(new_n270), .o1(new_n274));
  nand02aa1n02x5               g179(.a(new_n268), .b(new_n271), .o1(new_n275));
  inv000aa1d42x5               g180(.a(new_n272), .o1(new_n276));
  tech160nm_fiaoi012aa1n03p5x5 g181(.a(new_n276), .b(new_n275), .c(new_n274), .o1(new_n277));
  nor042aa1n03x5               g182(.a(new_n277), .b(new_n273), .o1(\s[26] ));
  inv000aa1d42x5               g183(.a(new_n151), .o1(new_n279));
  inv000aa1d42x5               g184(.a(new_n152), .o1(new_n280));
  aoai13aa1n02x5               g185(.a(new_n191), .b(new_n280), .c(new_n145), .d(new_n279), .o1(new_n281));
  nona22aa1n03x5               g186(.a(new_n281), .b(new_n192), .c(new_n193), .out0(new_n282));
  and002aa1n18x5               g187(.a(new_n272), .b(new_n271), .o(new_n283));
  nano22aa1n03x7               g188(.a(new_n244), .b(new_n283), .c(new_n262), .out0(new_n284));
  aoai13aa1n06x5               g189(.a(new_n284), .b(new_n282), .c(new_n168), .d(new_n188), .o1(new_n285));
  nona32aa1n09x5               g190(.a(new_n229), .b(new_n261), .c(new_n236), .d(new_n234), .out0(new_n286));
  nand02aa1d04x5               g191(.a(new_n286), .b(new_n265), .o1(new_n287));
  nanp02aa1n02x5               g192(.a(\b[25] ), .b(\a[26] ), .o1(new_n288));
  oai022aa1n02x5               g193(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n289));
  aoi022aa1n09x5               g194(.a(new_n287), .b(new_n283), .c(new_n288), .d(new_n289), .o1(new_n290));
  xorc02aa1n12x5               g195(.a(\a[27] ), .b(\b[26] ), .out0(new_n291));
  xnbna2aa1n03x5               g196(.a(new_n291), .b(new_n290), .c(new_n285), .out0(\s[27] ));
  nor042aa1n03x5               g197(.a(\b[26] ), .b(\a[27] ), .o1(new_n293));
  inv000aa1d42x5               g198(.a(new_n293), .o1(new_n294));
  inv000aa1d42x5               g199(.a(new_n291), .o1(new_n295));
  aoi012aa1n02x7               g200(.a(new_n295), .b(new_n290), .c(new_n285), .o1(new_n296));
  xnrc02aa1n12x5               g201(.a(\b[27] ), .b(\a[28] ), .out0(new_n297));
  nano22aa1n03x5               g202(.a(new_n296), .b(new_n294), .c(new_n297), .out0(new_n298));
  inv000aa1d42x5               g203(.a(new_n283), .o1(new_n299));
  nanp02aa1n02x5               g204(.a(new_n289), .b(new_n288), .o1(new_n300));
  aoai13aa1n04x5               g205(.a(new_n300), .b(new_n299), .c(new_n286), .d(new_n265), .o1(new_n301));
  aoai13aa1n03x5               g206(.a(new_n291), .b(new_n301), .c(new_n198), .d(new_n284), .o1(new_n302));
  tech160nm_fiaoi012aa1n03p5x5 g207(.a(new_n297), .b(new_n302), .c(new_n294), .o1(new_n303));
  norp02aa1n03x5               g208(.a(new_n303), .b(new_n298), .o1(\s[28] ));
  xnrc02aa1n02x5               g209(.a(\b[28] ), .b(\a[29] ), .out0(new_n305));
  norb02aa1d21x5               g210(.a(new_n291), .b(new_n297), .out0(new_n306));
  aoai13aa1n03x5               g211(.a(new_n306), .b(new_n301), .c(new_n198), .d(new_n284), .o1(new_n307));
  oao003aa1n02x5               g212(.a(\a[28] ), .b(\b[27] ), .c(new_n294), .carry(new_n308));
  tech160nm_fiaoi012aa1n02p5x5 g213(.a(new_n305), .b(new_n307), .c(new_n308), .o1(new_n309));
  inv000aa1d42x5               g214(.a(new_n306), .o1(new_n310));
  aoi012aa1n02x7               g215(.a(new_n310), .b(new_n290), .c(new_n285), .o1(new_n311));
  nano22aa1n03x5               g216(.a(new_n311), .b(new_n305), .c(new_n308), .out0(new_n312));
  norp02aa1n03x5               g217(.a(new_n309), .b(new_n312), .o1(\s[29] ));
  xorb03aa1n02x5               g218(.a(new_n98), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n12x5               g219(.a(new_n291), .b(new_n305), .c(new_n297), .out0(new_n315));
  aoai13aa1n03x5               g220(.a(new_n315), .b(new_n301), .c(new_n198), .d(new_n284), .o1(new_n316));
  oao003aa1n02x5               g221(.a(\a[29] ), .b(\b[28] ), .c(new_n308), .carry(new_n317));
  xnrc02aa1n02x5               g222(.a(\b[29] ), .b(\a[30] ), .out0(new_n318));
  tech160nm_fiaoi012aa1n03p5x5 g223(.a(new_n318), .b(new_n316), .c(new_n317), .o1(new_n319));
  inv000aa1d42x5               g224(.a(new_n315), .o1(new_n320));
  aoi012aa1n02x7               g225(.a(new_n320), .b(new_n290), .c(new_n285), .o1(new_n321));
  nano22aa1n03x5               g226(.a(new_n321), .b(new_n317), .c(new_n318), .out0(new_n322));
  norp02aa1n03x5               g227(.a(new_n319), .b(new_n322), .o1(\s[30] ));
  norb02aa1n06x5               g228(.a(new_n315), .b(new_n318), .out0(new_n324));
  inv000aa1d42x5               g229(.a(new_n324), .o1(new_n325));
  tech160nm_fiaoi012aa1n03p5x5 g230(.a(new_n325), .b(new_n290), .c(new_n285), .o1(new_n326));
  oao003aa1n02x5               g231(.a(\a[30] ), .b(\b[29] ), .c(new_n317), .carry(new_n327));
  xnrc02aa1n02x5               g232(.a(\b[30] ), .b(\a[31] ), .out0(new_n328));
  nano22aa1n03x5               g233(.a(new_n326), .b(new_n327), .c(new_n328), .out0(new_n329));
  aoai13aa1n03x5               g234(.a(new_n324), .b(new_n301), .c(new_n198), .d(new_n284), .o1(new_n330));
  tech160nm_fiaoi012aa1n02p5x5 g235(.a(new_n328), .b(new_n330), .c(new_n327), .o1(new_n331));
  norp02aa1n03x5               g236(.a(new_n331), .b(new_n329), .o1(\s[31] ));
  xnrb03aa1n02x5               g237(.a(new_n100), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g238(.a(\a[3] ), .b(\b[2] ), .c(new_n100), .o1(new_n334));
  xorb03aa1n02x5               g239(.a(new_n334), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g240(.a(new_n107), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oai012aa1n02x5               g241(.a(new_n114), .b(new_n107), .c(new_n113), .o1(new_n337));
  xorb03aa1n02x5               g242(.a(new_n337), .b(\b[5] ), .c(new_n119), .out0(\s[6] ));
  oaoi03aa1n02x5               g243(.a(\a[6] ), .b(\b[5] ), .c(new_n337), .o1(new_n339));
  xorb03aa1n02x5               g244(.a(new_n339), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  oai012aa1n02x5               g245(.a(new_n110), .b(new_n339), .c(new_n111), .o1(new_n341));
  xnrb03aa1n02x5               g246(.a(new_n341), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xorb03aa1n02x5               g247(.a(new_n168), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


