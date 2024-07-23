// Benchmark "adder" written by ABC on Thu Jul 18 03:32:13 2024

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
    new_n132, new_n133, new_n134, new_n135, new_n137, new_n138, new_n139,
    new_n140, new_n141, new_n142, new_n143, new_n145, new_n146, new_n147,
    new_n148, new_n149, new_n150, new_n151, new_n152, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n161, new_n162,
    new_n163, new_n164, new_n165, new_n166, new_n168, new_n169, new_n170,
    new_n172, new_n173, new_n174, new_n175, new_n176, new_n177, new_n178,
    new_n180, new_n181, new_n182, new_n183, new_n184, new_n185, new_n186,
    new_n187, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n197, new_n198, new_n199, new_n200, new_n201,
    new_n202, new_n204, new_n205, new_n206, new_n208, new_n209, new_n210,
    new_n211, new_n212, new_n213, new_n214, new_n217, new_n218, new_n219,
    new_n220, new_n221, new_n222, new_n223, new_n224, new_n225, new_n226,
    new_n227, new_n229, new_n230, new_n231, new_n232, new_n233, new_n234,
    new_n235, new_n236, new_n237, new_n238, new_n239, new_n240, new_n241,
    new_n242, new_n243, new_n244, new_n246, new_n247, new_n248, new_n249,
    new_n250, new_n251, new_n252, new_n254, new_n255, new_n256, new_n257,
    new_n258, new_n259, new_n260, new_n261, new_n262, new_n263, new_n265,
    new_n266, new_n267, new_n268, new_n269, new_n270, new_n272, new_n273,
    new_n274, new_n275, new_n276, new_n277, new_n278, new_n279, new_n280,
    new_n281, new_n282, new_n283, new_n284, new_n286, new_n287, new_n288,
    new_n289, new_n290, new_n291, new_n293, new_n294, new_n295, new_n296,
    new_n297, new_n298, new_n299, new_n300, new_n301, new_n302, new_n304,
    new_n305, new_n306, new_n307, new_n308, new_n309, new_n310, new_n312,
    new_n313, new_n314, new_n315, new_n316, new_n317, new_n318, new_n321,
    new_n322, new_n323, new_n324, new_n325, new_n326, new_n327, new_n328,
    new_n329, new_n330, new_n332, new_n333, new_n334, new_n335, new_n336,
    new_n337, new_n338, new_n339, new_n340, new_n341, new_n342, new_n343,
    new_n345, new_n346, new_n348, new_n349, new_n350, new_n351, new_n352,
    new_n354, new_n355, new_n356, new_n357, new_n359, new_n361, new_n362,
    new_n363, new_n364, new_n365, new_n366, new_n368, new_n370;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor022aa1n08x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(new_n97), .o1(new_n98));
  inv000aa1d42x5               g003(.a(\a[2] ), .o1(new_n99));
  inv000aa1d42x5               g004(.a(\b[1] ), .o1(new_n100));
  nanp02aa1n02x5               g005(.a(new_n100), .b(new_n99), .o1(new_n101));
  nand22aa1n02x5               g006(.a(\b[0] ), .b(\a[1] ), .o1(new_n102));
  aoi012aa1n06x5               g007(.a(new_n102), .b(\a[2] ), .c(\b[1] ), .o1(new_n103));
  nand42aa1n02x5               g008(.a(\b[1] ), .b(\a[2] ), .o1(new_n104));
  tech160nm_finand02aa1n03p5x5 g009(.a(\b[2] ), .b(\a[3] ), .o1(new_n105));
  nor002aa1n04x5               g010(.a(\b[2] ), .b(\a[3] ), .o1(new_n106));
  nanb03aa1n03x5               g011(.a(new_n106), .b(new_n104), .c(new_n105), .out0(new_n107));
  tech160nm_fiaoi012aa1n05x5   g012(.a(new_n107), .b(new_n101), .c(new_n103), .o1(new_n108));
  oai022aa1n02x5               g013(.a(\a[3] ), .b(\b[2] ), .c(\b[3] ), .d(\a[4] ), .o1(new_n109));
  nanp02aa1n02x5               g014(.a(\b[4] ), .b(\a[5] ), .o1(new_n110));
  nanp02aa1n02x5               g015(.a(\b[5] ), .b(\a[6] ), .o1(new_n111));
  nor042aa1n12x5               g016(.a(\b[4] ), .b(\a[5] ), .o1(new_n112));
  nanb03aa1n03x5               g017(.a(new_n112), .b(new_n110), .c(new_n111), .out0(new_n113));
  nanp02aa1n02x5               g018(.a(\b[3] ), .b(\a[4] ), .o1(new_n114));
  nor042aa1n02x5               g019(.a(\b[7] ), .b(\a[8] ), .o1(new_n115));
  nand02aa1n02x5               g020(.a(\b[7] ), .b(\a[8] ), .o1(new_n116));
  nanb03aa1n02x5               g021(.a(new_n115), .b(new_n116), .c(new_n114), .out0(new_n117));
  inv040aa1d32x5               g022(.a(\a[7] ), .o1(new_n118));
  inv000aa1d42x5               g023(.a(\b[6] ), .o1(new_n119));
  nand02aa1d08x5               g024(.a(new_n119), .b(new_n118), .o1(new_n120));
  nanp02aa1n02x5               g025(.a(\b[6] ), .b(\a[7] ), .o1(new_n121));
  oai112aa1n02x5               g026(.a(new_n120), .b(new_n121), .c(\b[5] ), .d(\a[6] ), .o1(new_n122));
  nor043aa1n03x5               g027(.a(new_n122), .b(new_n117), .c(new_n113), .o1(new_n123));
  oai012aa1n09x5               g028(.a(new_n123), .b(new_n108), .c(new_n109), .o1(new_n124));
  oaoi03aa1n02x5               g029(.a(\a[8] ), .b(\b[7] ), .c(new_n120), .o1(new_n125));
  nanp03aa1n03x5               g030(.a(new_n116), .b(new_n111), .c(new_n121), .o1(new_n126));
  oai022aa1n03x5               g031(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n127));
  nano23aa1n02x4               g032(.a(new_n126), .b(new_n115), .c(new_n127), .d(new_n120), .out0(new_n128));
  nor022aa1n03x5               g033(.a(new_n128), .b(new_n125), .o1(new_n129));
  nanp02aa1n06x5               g034(.a(new_n124), .b(new_n129), .o1(new_n130));
  xnrc02aa1n12x5               g035(.a(\b[8] ), .b(\a[9] ), .out0(new_n131));
  inv000aa1d42x5               g036(.a(new_n131), .o1(new_n132));
  nanp02aa1n02x5               g037(.a(new_n130), .b(new_n132), .o1(new_n133));
  tech160nm_fixnrc02aa1n04x5   g038(.a(\b[9] ), .b(\a[10] ), .out0(new_n134));
  inv000aa1n02x5               g039(.a(new_n134), .o1(new_n135));
  xnbna2aa1n03x5               g040(.a(new_n135), .b(new_n133), .c(new_n98), .out0(\s[10] ));
  aoai13aa1n06x5               g041(.a(new_n135), .b(new_n97), .c(new_n130), .d(new_n132), .o1(new_n137));
  nanp02aa1n02x5               g042(.a(\b[9] ), .b(\a[10] ), .o1(new_n138));
  oaih22aa1d12x5               g043(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n139));
  nanp02aa1n02x5               g044(.a(new_n139), .b(new_n138), .o1(new_n140));
  nor002aa1n16x5               g045(.a(\b[10] ), .b(\a[11] ), .o1(new_n141));
  nand42aa1n03x5               g046(.a(\b[10] ), .b(\a[11] ), .o1(new_n142));
  norb02aa1n09x5               g047(.a(new_n142), .b(new_n141), .out0(new_n143));
  xnbna2aa1n03x5               g048(.a(new_n143), .b(new_n137), .c(new_n140), .out0(\s[11] ));
  aob012aa1n02x5               g049(.a(new_n143), .b(new_n137), .c(new_n140), .out0(new_n145));
  inv000aa1d42x5               g050(.a(new_n141), .o1(new_n146));
  inv000aa1d42x5               g051(.a(new_n143), .o1(new_n147));
  aoai13aa1n02x5               g052(.a(new_n146), .b(new_n147), .c(new_n137), .d(new_n140), .o1(new_n148));
  nor042aa1n06x5               g053(.a(\b[11] ), .b(\a[12] ), .o1(new_n149));
  nand02aa1n06x5               g054(.a(\b[11] ), .b(\a[12] ), .o1(new_n150));
  nanb02aa1n02x5               g055(.a(new_n149), .b(new_n150), .out0(new_n151));
  aoib12aa1n02x5               g056(.a(new_n141), .b(new_n150), .c(new_n149), .out0(new_n152));
  aboi22aa1n03x5               g057(.a(new_n151), .b(new_n148), .c(new_n145), .d(new_n152), .out0(\s[12] ));
  nona23aa1n02x4               g058(.a(new_n150), .b(new_n142), .c(new_n141), .d(new_n149), .out0(new_n154));
  nor043aa1n03x5               g059(.a(new_n154), .b(new_n134), .c(new_n131), .o1(new_n155));
  nano22aa1n03x7               g060(.a(new_n149), .b(new_n142), .c(new_n150), .out0(new_n156));
  oai112aa1n03x5               g061(.a(new_n139), .b(new_n138), .c(\b[10] ), .d(\a[11] ), .o1(new_n157));
  aoi012aa1n02x5               g062(.a(new_n149), .b(new_n141), .c(new_n150), .o1(new_n158));
  oaib12aa1n03x5               g063(.a(new_n158), .b(new_n157), .c(new_n156), .out0(new_n159));
  xnrc02aa1n12x5               g064(.a(\b[12] ), .b(\a[13] ), .out0(new_n160));
  inv000aa1d42x5               g065(.a(new_n160), .o1(new_n161));
  aoai13aa1n02x5               g066(.a(new_n161), .b(new_n159), .c(new_n130), .d(new_n155), .o1(new_n162));
  inv000aa1n02x5               g067(.a(new_n157), .o1(new_n163));
  oaoi03aa1n02x5               g068(.a(\a[12] ), .b(\b[11] ), .c(new_n146), .o1(new_n164));
  aoi112aa1n02x5               g069(.a(new_n164), .b(new_n161), .c(new_n163), .d(new_n156), .o1(new_n165));
  aobi12aa1n02x5               g070(.a(new_n165), .b(new_n130), .c(new_n155), .out0(new_n166));
  norb02aa1n02x5               g071(.a(new_n162), .b(new_n166), .out0(\s[13] ));
  norp02aa1n02x5               g072(.a(\b[12] ), .b(\a[13] ), .o1(new_n168));
  inv000aa1n03x5               g073(.a(new_n168), .o1(new_n169));
  tech160nm_fixnrc02aa1n05x5   g074(.a(\b[13] ), .b(\a[14] ), .out0(new_n170));
  xobna2aa1n03x5               g075(.a(new_n170), .b(new_n162), .c(new_n169), .out0(\s[14] ));
  nor042aa1n02x5               g076(.a(new_n170), .b(new_n160), .o1(new_n172));
  aoai13aa1n03x5               g077(.a(new_n172), .b(new_n159), .c(new_n130), .d(new_n155), .o1(new_n173));
  oaoi03aa1n02x5               g078(.a(\a[14] ), .b(\b[13] ), .c(new_n169), .o1(new_n174));
  inv000aa1n02x5               g079(.a(new_n174), .o1(new_n175));
  nor042aa1n12x5               g080(.a(\b[14] ), .b(\a[15] ), .o1(new_n176));
  nand42aa1n03x5               g081(.a(\b[14] ), .b(\a[15] ), .o1(new_n177));
  norb02aa1n02x5               g082(.a(new_n177), .b(new_n176), .out0(new_n178));
  xnbna2aa1n03x5               g083(.a(new_n178), .b(new_n173), .c(new_n175), .out0(\s[15] ));
  aob012aa1n06x5               g084(.a(new_n178), .b(new_n173), .c(new_n175), .out0(new_n180));
  inv000aa1d42x5               g085(.a(new_n176), .o1(new_n181));
  inv000aa1n02x5               g086(.a(new_n178), .o1(new_n182));
  aoai13aa1n02x5               g087(.a(new_n181), .b(new_n182), .c(new_n173), .d(new_n175), .o1(new_n183));
  nor002aa1n03x5               g088(.a(\b[15] ), .b(\a[16] ), .o1(new_n184));
  nanp02aa1n04x5               g089(.a(\b[15] ), .b(\a[16] ), .o1(new_n185));
  norb02aa1n02x5               g090(.a(new_n185), .b(new_n184), .out0(new_n186));
  aoib12aa1n02x5               g091(.a(new_n176), .b(new_n185), .c(new_n184), .out0(new_n187));
  aoi022aa1n03x5               g092(.a(new_n183), .b(new_n186), .c(new_n180), .d(new_n187), .o1(\s[16] ));
  nona23aa1n09x5               g093(.a(new_n185), .b(new_n177), .c(new_n176), .d(new_n184), .out0(new_n189));
  aoai13aa1n06x5               g094(.a(new_n172), .b(new_n164), .c(new_n163), .d(new_n156), .o1(new_n190));
  aoi012aa1n12x5               g095(.a(new_n189), .b(new_n190), .c(new_n175), .o1(new_n191));
  nona32aa1n03x5               g096(.a(new_n155), .b(new_n189), .c(new_n170), .d(new_n160), .out0(new_n192));
  tech160nm_fiaoi012aa1n03p5x5 g097(.a(new_n184), .b(new_n176), .c(new_n185), .o1(new_n193));
  aoai13aa1n04x5               g098(.a(new_n193), .b(new_n192), .c(new_n124), .d(new_n129), .o1(new_n194));
  tech160nm_fixorc02aa1n03p5x5 g099(.a(\a[17] ), .b(\b[16] ), .out0(new_n195));
  tech160nm_fioai012aa1n05x5   g100(.a(new_n195), .b(new_n194), .c(new_n191), .o1(new_n196));
  inv000aa1n02x5               g101(.a(new_n189), .o1(new_n197));
  aoai13aa1n06x5               g102(.a(new_n197), .b(new_n174), .c(new_n159), .d(new_n172), .o1(new_n198));
  nona23aa1n06x5               g103(.a(new_n135), .b(new_n143), .c(new_n131), .d(new_n151), .out0(new_n199));
  nano22aa1n03x7               g104(.a(new_n199), .b(new_n172), .c(new_n197), .out0(new_n200));
  nanp02aa1n09x5               g105(.a(new_n130), .b(new_n200), .o1(new_n201));
  nano32aa1n02x4               g106(.a(new_n195), .b(new_n201), .c(new_n198), .d(new_n193), .out0(new_n202));
  norb02aa1n02x5               g107(.a(new_n196), .b(new_n202), .out0(\s[17] ));
  inv000aa1d42x5               g108(.a(\a[17] ), .o1(new_n204));
  nanb02aa1n02x5               g109(.a(\b[16] ), .b(new_n204), .out0(new_n205));
  xorc02aa1n02x5               g110(.a(\a[18] ), .b(\b[17] ), .out0(new_n206));
  xnbna2aa1n03x5               g111(.a(new_n206), .b(new_n196), .c(new_n205), .out0(\s[18] ));
  and002aa1n02x5               g112(.a(new_n206), .b(new_n195), .o(new_n208));
  tech160nm_fioai012aa1n05x5   g113(.a(new_n208), .b(new_n194), .c(new_n191), .o1(new_n209));
  oaoi03aa1n02x5               g114(.a(\a[18] ), .b(\b[17] ), .c(new_n205), .o1(new_n210));
  inv000aa1d42x5               g115(.a(new_n210), .o1(new_n211));
  nor042aa1d18x5               g116(.a(\b[18] ), .b(\a[19] ), .o1(new_n212));
  nand02aa1d16x5               g117(.a(\b[18] ), .b(\a[19] ), .o1(new_n213));
  norb02aa1n03x5               g118(.a(new_n213), .b(new_n212), .out0(new_n214));
  xnbna2aa1n03x5               g119(.a(new_n214), .b(new_n209), .c(new_n211), .out0(\s[19] ));
  xnrc02aa1n02x5               g120(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nand23aa1n06x5               g121(.a(new_n201), .b(new_n198), .c(new_n193), .o1(new_n217));
  aoai13aa1n03x5               g122(.a(new_n214), .b(new_n210), .c(new_n217), .d(new_n208), .o1(new_n218));
  inv000aa1d42x5               g123(.a(new_n212), .o1(new_n219));
  inv040aa1n03x5               g124(.a(new_n214), .o1(new_n220));
  aoai13aa1n02x5               g125(.a(new_n219), .b(new_n220), .c(new_n209), .d(new_n211), .o1(new_n221));
  nor042aa1n09x5               g126(.a(\b[19] ), .b(\a[20] ), .o1(new_n222));
  nand22aa1n12x5               g127(.a(\b[19] ), .b(\a[20] ), .o1(new_n223));
  norb02aa1n02x5               g128(.a(new_n223), .b(new_n222), .out0(new_n224));
  inv000aa1d42x5               g129(.a(\a[19] ), .o1(new_n225));
  inv000aa1d42x5               g130(.a(\b[18] ), .o1(new_n226));
  aboi22aa1n03x5               g131(.a(new_n222), .b(new_n223), .c(new_n225), .d(new_n226), .out0(new_n227));
  aoi022aa1n03x5               g132(.a(new_n221), .b(new_n224), .c(new_n218), .d(new_n227), .o1(\s[20] ));
  nano32aa1n03x7               g133(.a(new_n220), .b(new_n206), .c(new_n195), .d(new_n224), .out0(new_n229));
  oai012aa1n03x5               g134(.a(new_n229), .b(new_n194), .c(new_n191), .o1(new_n230));
  nanb03aa1n09x5               g135(.a(new_n222), .b(new_n223), .c(new_n213), .out0(new_n231));
  nand02aa1d06x5               g136(.a(\b[17] ), .b(\a[18] ), .o1(new_n232));
  oaih22aa1d12x5               g137(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n233));
  oai112aa1n06x5               g138(.a(new_n233), .b(new_n232), .c(\b[18] ), .d(\a[19] ), .o1(new_n234));
  aoi012aa1n09x5               g139(.a(new_n222), .b(new_n212), .c(new_n223), .o1(new_n235));
  oai012aa1n18x5               g140(.a(new_n235), .b(new_n234), .c(new_n231), .o1(new_n236));
  xnrc02aa1n12x5               g141(.a(\b[20] ), .b(\a[21] ), .out0(new_n237));
  inv000aa1d42x5               g142(.a(new_n237), .o1(new_n238));
  aoai13aa1n06x5               g143(.a(new_n238), .b(new_n236), .c(new_n217), .d(new_n229), .o1(new_n239));
  nano22aa1n03x5               g144(.a(new_n222), .b(new_n213), .c(new_n223), .out0(new_n240));
  oai012aa1n02x5               g145(.a(new_n232), .b(\b[18] ), .c(\a[19] ), .o1(new_n241));
  norb02aa1n02x5               g146(.a(new_n233), .b(new_n241), .out0(new_n242));
  inv000aa1n02x5               g147(.a(new_n235), .o1(new_n243));
  aoi112aa1n02x5               g148(.a(new_n243), .b(new_n238), .c(new_n242), .d(new_n240), .o1(new_n244));
  aobi12aa1n03x7               g149(.a(new_n239), .b(new_n244), .c(new_n230), .out0(\s[21] ));
  inv000aa1d42x5               g150(.a(new_n236), .o1(new_n246));
  nor042aa1n03x5               g151(.a(\b[20] ), .b(\a[21] ), .o1(new_n247));
  inv000aa1n03x5               g152(.a(new_n247), .o1(new_n248));
  aoai13aa1n03x5               g153(.a(new_n248), .b(new_n237), .c(new_n230), .d(new_n246), .o1(new_n249));
  xnrc02aa1n12x5               g154(.a(\b[21] ), .b(\a[22] ), .out0(new_n250));
  inv000aa1d42x5               g155(.a(new_n250), .o1(new_n251));
  norb02aa1n02x5               g156(.a(new_n250), .b(new_n247), .out0(new_n252));
  aoi022aa1n03x5               g157(.a(new_n249), .b(new_n251), .c(new_n239), .d(new_n252), .o1(\s[22] ));
  nor042aa1n06x5               g158(.a(new_n250), .b(new_n237), .o1(new_n254));
  and002aa1n02x5               g159(.a(new_n229), .b(new_n254), .o(new_n255));
  oai012aa1n03x5               g160(.a(new_n255), .b(new_n194), .c(new_n191), .o1(new_n256));
  oao003aa1n02x5               g161(.a(\a[22] ), .b(\b[21] ), .c(new_n248), .carry(new_n257));
  inv030aa1n02x5               g162(.a(new_n257), .o1(new_n258));
  tech160nm_fiaoi012aa1n03p5x5 g163(.a(new_n258), .b(new_n236), .c(new_n254), .o1(new_n259));
  inv000aa1n02x5               g164(.a(new_n259), .o1(new_n260));
  xorc02aa1n12x5               g165(.a(\a[23] ), .b(\b[22] ), .out0(new_n261));
  aoai13aa1n06x5               g166(.a(new_n261), .b(new_n260), .c(new_n217), .d(new_n255), .o1(new_n262));
  aoi112aa1n02x5               g167(.a(new_n261), .b(new_n258), .c(new_n236), .d(new_n254), .o1(new_n263));
  aobi12aa1n03x7               g168(.a(new_n262), .b(new_n263), .c(new_n256), .out0(\s[23] ));
  nor042aa1n06x5               g169(.a(\b[22] ), .b(\a[23] ), .o1(new_n265));
  inv000aa1d42x5               g170(.a(new_n265), .o1(new_n266));
  inv000aa1d42x5               g171(.a(new_n261), .o1(new_n267));
  aoai13aa1n03x5               g172(.a(new_n266), .b(new_n267), .c(new_n256), .d(new_n259), .o1(new_n268));
  xorc02aa1n02x5               g173(.a(\a[24] ), .b(\b[23] ), .out0(new_n269));
  norp02aa1n02x5               g174(.a(new_n269), .b(new_n265), .o1(new_n270));
  aoi022aa1n02x5               g175(.a(new_n268), .b(new_n269), .c(new_n262), .d(new_n270), .o1(\s[24] ));
  inv000aa1n02x5               g176(.a(new_n229), .o1(new_n272));
  and002aa1n12x5               g177(.a(new_n269), .b(new_n261), .o(new_n273));
  nano22aa1n03x7               g178(.a(new_n272), .b(new_n273), .c(new_n254), .out0(new_n274));
  oai012aa1n02x7               g179(.a(new_n274), .b(new_n194), .c(new_n191), .o1(new_n275));
  aoai13aa1n06x5               g180(.a(new_n254), .b(new_n243), .c(new_n242), .d(new_n240), .o1(new_n276));
  inv020aa1n02x5               g181(.a(new_n273), .o1(new_n277));
  oao003aa1n02x5               g182(.a(\a[24] ), .b(\b[23] ), .c(new_n266), .carry(new_n278));
  aoai13aa1n12x5               g183(.a(new_n278), .b(new_n277), .c(new_n276), .d(new_n257), .o1(new_n279));
  xorc02aa1n12x5               g184(.a(\a[25] ), .b(\b[24] ), .out0(new_n280));
  aoai13aa1n06x5               g185(.a(new_n280), .b(new_n279), .c(new_n217), .d(new_n274), .o1(new_n281));
  aoai13aa1n04x5               g186(.a(new_n273), .b(new_n258), .c(new_n236), .d(new_n254), .o1(new_n282));
  inv000aa1d42x5               g187(.a(new_n280), .o1(new_n283));
  and003aa1n02x5               g188(.a(new_n282), .b(new_n283), .c(new_n278), .o(new_n284));
  aobi12aa1n03x7               g189(.a(new_n281), .b(new_n284), .c(new_n275), .out0(\s[25] ));
  inv000aa1d42x5               g190(.a(new_n279), .o1(new_n286));
  nor042aa1n03x5               g191(.a(\b[24] ), .b(\a[25] ), .o1(new_n287));
  inv000aa1d42x5               g192(.a(new_n287), .o1(new_n288));
  aoai13aa1n03x5               g193(.a(new_n288), .b(new_n283), .c(new_n275), .d(new_n286), .o1(new_n289));
  xorc02aa1n02x5               g194(.a(\a[26] ), .b(\b[25] ), .out0(new_n290));
  norp02aa1n02x5               g195(.a(new_n290), .b(new_n287), .o1(new_n291));
  aoi022aa1n03x5               g196(.a(new_n289), .b(new_n290), .c(new_n281), .d(new_n291), .o1(\s[26] ));
  and002aa1n12x5               g197(.a(new_n290), .b(new_n280), .o(new_n293));
  nano32aa1n03x7               g198(.a(new_n272), .b(new_n293), .c(new_n254), .d(new_n273), .out0(new_n294));
  oai012aa1n09x5               g199(.a(new_n294), .b(new_n194), .c(new_n191), .o1(new_n295));
  inv000aa1d42x5               g200(.a(new_n293), .o1(new_n296));
  oao003aa1n02x5               g201(.a(\a[26] ), .b(\b[25] ), .c(new_n288), .carry(new_n297));
  aoai13aa1n06x5               g202(.a(new_n297), .b(new_n296), .c(new_n282), .d(new_n278), .o1(new_n298));
  tech160nm_fixorc02aa1n04x5   g203(.a(\a[27] ), .b(\b[26] ), .out0(new_n299));
  aoai13aa1n03x5               g204(.a(new_n299), .b(new_n298), .c(new_n217), .d(new_n294), .o1(new_n300));
  inv000aa1d42x5               g205(.a(new_n297), .o1(new_n301));
  aoi112aa1n02x5               g206(.a(new_n299), .b(new_n301), .c(new_n279), .d(new_n293), .o1(new_n302));
  aobi12aa1n03x7               g207(.a(new_n300), .b(new_n302), .c(new_n295), .out0(\s[27] ));
  aoi012aa1d18x5               g208(.a(new_n301), .b(new_n279), .c(new_n293), .o1(new_n304));
  norp02aa1n02x5               g209(.a(\b[26] ), .b(\a[27] ), .o1(new_n305));
  inv000aa1n03x5               g210(.a(new_n305), .o1(new_n306));
  inv000aa1n02x5               g211(.a(new_n299), .o1(new_n307));
  aoai13aa1n06x5               g212(.a(new_n306), .b(new_n307), .c(new_n295), .d(new_n304), .o1(new_n308));
  tech160nm_fixorc02aa1n03p5x5 g213(.a(\a[28] ), .b(\b[27] ), .out0(new_n309));
  norp02aa1n02x5               g214(.a(new_n309), .b(new_n305), .o1(new_n310));
  aoi022aa1n03x5               g215(.a(new_n308), .b(new_n309), .c(new_n300), .d(new_n310), .o1(\s[28] ));
  and002aa1n02x5               g216(.a(new_n309), .b(new_n299), .o(new_n312));
  aoai13aa1n03x5               g217(.a(new_n312), .b(new_n298), .c(new_n217), .d(new_n294), .o1(new_n313));
  inv000aa1d42x5               g218(.a(new_n312), .o1(new_n314));
  oao003aa1n02x5               g219(.a(\a[28] ), .b(\b[27] ), .c(new_n306), .carry(new_n315));
  aoai13aa1n06x5               g220(.a(new_n315), .b(new_n314), .c(new_n295), .d(new_n304), .o1(new_n316));
  tech160nm_fixorc02aa1n02p5x5 g221(.a(\a[29] ), .b(\b[28] ), .out0(new_n317));
  norb02aa1n02x5               g222(.a(new_n315), .b(new_n317), .out0(new_n318));
  aoi022aa1n03x5               g223(.a(new_n316), .b(new_n317), .c(new_n313), .d(new_n318), .o1(\s[29] ));
  xorb03aa1n02x5               g224(.a(new_n102), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n02x4               g225(.a(new_n307), .b(new_n309), .c(new_n317), .out0(new_n321));
  aoai13aa1n02x5               g226(.a(new_n321), .b(new_n298), .c(new_n217), .d(new_n294), .o1(new_n322));
  inv000aa1n02x5               g227(.a(new_n321), .o1(new_n323));
  inv000aa1d42x5               g228(.a(\b[28] ), .o1(new_n324));
  inv000aa1d42x5               g229(.a(\a[29] ), .o1(new_n325));
  oaib12aa1n02x5               g230(.a(new_n315), .b(\b[28] ), .c(new_n325), .out0(new_n326));
  oaib12aa1n02x5               g231(.a(new_n326), .b(new_n324), .c(\a[29] ), .out0(new_n327));
  aoai13aa1n06x5               g232(.a(new_n327), .b(new_n323), .c(new_n295), .d(new_n304), .o1(new_n328));
  xorc02aa1n02x5               g233(.a(\a[30] ), .b(\b[29] ), .out0(new_n329));
  oaoi13aa1n02x5               g234(.a(new_n329), .b(new_n326), .c(new_n325), .d(new_n324), .o1(new_n330));
  aoi022aa1n03x5               g235(.a(new_n328), .b(new_n329), .c(new_n322), .d(new_n330), .o1(\s[30] ));
  nanb02aa1n02x5               g236(.a(\b[30] ), .b(\a[31] ), .out0(new_n332));
  nanb02aa1n02x5               g237(.a(\a[31] ), .b(\b[30] ), .out0(new_n333));
  nanp02aa1n02x5               g238(.a(new_n333), .b(new_n332), .o1(new_n334));
  nano32aa1n02x4               g239(.a(new_n307), .b(new_n329), .c(new_n309), .d(new_n317), .out0(new_n335));
  aoai13aa1n02x5               g240(.a(new_n335), .b(new_n298), .c(new_n217), .d(new_n294), .o1(new_n336));
  inv000aa1n02x5               g241(.a(new_n335), .o1(new_n337));
  norp02aa1n02x5               g242(.a(\b[29] ), .b(\a[30] ), .o1(new_n338));
  aoi022aa1n02x5               g243(.a(\b[29] ), .b(\a[30] ), .c(\a[29] ), .d(\b[28] ), .o1(new_n339));
  aoi012aa1n02x5               g244(.a(new_n338), .b(new_n326), .c(new_n339), .o1(new_n340));
  aoai13aa1n06x5               g245(.a(new_n340), .b(new_n337), .c(new_n295), .d(new_n304), .o1(new_n341));
  oai112aa1n02x5               g246(.a(new_n332), .b(new_n333), .c(\b[29] ), .d(\a[30] ), .o1(new_n342));
  aoi012aa1n02x5               g247(.a(new_n342), .b(new_n326), .c(new_n339), .o1(new_n343));
  aoi022aa1n03x5               g248(.a(new_n341), .b(new_n334), .c(new_n336), .d(new_n343), .o1(\s[31] ));
  nanb02aa1n02x5               g249(.a(new_n106), .b(new_n105), .out0(new_n345));
  oaoi03aa1n02x5               g250(.a(new_n99), .b(new_n100), .c(new_n102), .o1(new_n346));
  aoi012aa1n02x5               g251(.a(new_n108), .b(new_n345), .c(new_n346), .o1(\s[3] ));
  nano22aa1n02x4               g252(.a(new_n106), .b(new_n104), .c(new_n105), .out0(new_n348));
  aob012aa1n02x5               g253(.a(new_n348), .b(new_n103), .c(new_n101), .out0(new_n349));
  nanb02aa1n02x5               g254(.a(new_n109), .b(new_n349), .out0(new_n350));
  xorc02aa1n02x5               g255(.a(\a[4] ), .b(\b[3] ), .out0(new_n351));
  norp02aa1n02x5               g256(.a(new_n351), .b(new_n106), .o1(new_n352));
  aoi022aa1n02x5               g257(.a(new_n350), .b(new_n351), .c(new_n349), .d(new_n352), .o1(\s[4] ));
  nano22aa1n02x4               g258(.a(new_n112), .b(new_n110), .c(new_n114), .out0(new_n354));
  oai012aa1n02x5               g259(.a(new_n354), .b(new_n108), .c(new_n109), .o1(new_n355));
  inv000aa1d42x5               g260(.a(new_n112), .o1(new_n356));
  aoi022aa1n02x5               g261(.a(new_n350), .b(new_n114), .c(new_n110), .d(new_n356), .o1(new_n357));
  norb02aa1n02x5               g262(.a(new_n355), .b(new_n357), .out0(\s[5] ));
  xorc02aa1n02x5               g263(.a(\a[6] ), .b(\b[5] ), .out0(new_n359));
  xnbna2aa1n03x5               g264(.a(new_n359), .b(new_n355), .c(new_n356), .out0(\s[6] ));
  inv000aa1d42x5               g265(.a(\a[6] ), .o1(new_n361));
  inv000aa1d42x5               g266(.a(\b[5] ), .o1(new_n362));
  aobi12aa1n02x5               g267(.a(new_n359), .b(new_n355), .c(new_n356), .out0(new_n363));
  xorc02aa1n02x5               g268(.a(\a[7] ), .b(\b[6] ), .out0(new_n364));
  aoai13aa1n02x5               g269(.a(new_n364), .b(new_n363), .c(new_n361), .d(new_n362), .o1(new_n365));
  aoi112aa1n02x5               g270(.a(new_n363), .b(new_n364), .c(new_n361), .d(new_n362), .o1(new_n366));
  norb02aa1n02x5               g271(.a(new_n365), .b(new_n366), .out0(\s[7] ));
  norb02aa1n02x5               g272(.a(new_n116), .b(new_n115), .out0(new_n368));
  xnbna2aa1n03x5               g273(.a(new_n368), .b(new_n365), .c(new_n120), .out0(\s[8] ));
  norp03aa1n02x5               g274(.a(new_n128), .b(new_n132), .c(new_n125), .o1(new_n370));
  aoi022aa1n02x5               g275(.a(new_n130), .b(new_n132), .c(new_n124), .d(new_n370), .o1(\s[9] ));
endmodule


