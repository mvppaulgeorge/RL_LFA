// Benchmark "adder" written by ABC on Thu Jul 18 12:50:59 2024

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
    new_n140, new_n141, new_n142, new_n143, new_n144, new_n146, new_n147,
    new_n148, new_n149, new_n150, new_n151, new_n152, new_n153, new_n154,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n161, new_n162,
    new_n163, new_n164, new_n165, new_n166, new_n167, new_n168, new_n169,
    new_n170, new_n171, new_n173, new_n174, new_n175, new_n176, new_n178,
    new_n179, new_n180, new_n181, new_n182, new_n183, new_n184, new_n185,
    new_n186, new_n188, new_n189, new_n190, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n197, new_n198, new_n199, new_n200, new_n201,
    new_n203, new_n204, new_n205, new_n206, new_n207, new_n208, new_n209,
    new_n211, new_n212, new_n213, new_n214, new_n215, new_n216, new_n217,
    new_n220, new_n221, new_n222, new_n223, new_n224, new_n225, new_n226,
    new_n227, new_n228, new_n229, new_n231, new_n232, new_n233, new_n234,
    new_n235, new_n236, new_n237, new_n238, new_n239, new_n240, new_n241,
    new_n242, new_n243, new_n244, new_n245, new_n246, new_n248, new_n249,
    new_n250, new_n251, new_n252, new_n253, new_n254, new_n255, new_n257,
    new_n258, new_n259, new_n260, new_n261, new_n262, new_n263, new_n264,
    new_n265, new_n266, new_n268, new_n269, new_n270, new_n271, new_n272,
    new_n273, new_n274, new_n275, new_n277, new_n278, new_n279, new_n280,
    new_n281, new_n282, new_n283, new_n284, new_n285, new_n286, new_n287,
    new_n288, new_n289, new_n291, new_n292, new_n293, new_n294, new_n295,
    new_n296, new_n298, new_n299, new_n300, new_n301, new_n302, new_n303,
    new_n304, new_n305, new_n306, new_n308, new_n309, new_n310, new_n311,
    new_n312, new_n313, new_n314, new_n315, new_n316, new_n318, new_n319,
    new_n320, new_n321, new_n322, new_n323, new_n324, new_n327, new_n328,
    new_n329, new_n330, new_n331, new_n332, new_n333, new_n335, new_n336,
    new_n337, new_n338, new_n339, new_n340, new_n341, new_n343, new_n345,
    new_n346, new_n349, new_n350, new_n351, new_n352, new_n353, new_n355,
    new_n357, new_n358, new_n359, new_n361, new_n362;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor042aa1n09x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  inv000aa1n02x5               g002(.a(new_n97), .o1(new_n98));
  nor022aa1n06x5               g003(.a(\b[1] ), .b(\a[2] ), .o1(new_n99));
  nand02aa1d16x5               g004(.a(\b[1] ), .b(\a[2] ), .o1(new_n100));
  nand22aa1n04x5               g005(.a(\b[0] ), .b(\a[1] ), .o1(new_n101));
  norb03aa1n09x5               g006(.a(new_n100), .b(new_n99), .c(new_n101), .out0(new_n102));
  tech160nm_fioai012aa1n04x5   g007(.a(new_n100), .b(\b[3] ), .c(\a[4] ), .o1(new_n103));
  tech160nm_finand02aa1n05x5   g008(.a(\b[2] ), .b(\a[3] ), .o1(new_n104));
  tech160nm_finand02aa1n05x5   g009(.a(\b[3] ), .b(\a[4] ), .o1(new_n105));
  norp02aa1n12x5               g010(.a(\b[2] ), .b(\a[3] ), .o1(new_n106));
  nanb03aa1n12x5               g011(.a(new_n106), .b(new_n104), .c(new_n105), .out0(new_n107));
  norp02aa1n04x5               g012(.a(\b[3] ), .b(\a[4] ), .o1(new_n108));
  tech160nm_fioai012aa1n03p5x5 g013(.a(new_n105), .b(new_n106), .c(new_n108), .o1(new_n109));
  oai013aa1n06x5               g014(.a(new_n109), .b(new_n102), .c(new_n107), .d(new_n103), .o1(new_n110));
  xorc02aa1n02x5               g015(.a(\a[6] ), .b(\b[5] ), .out0(new_n111));
  xorc02aa1n12x5               g016(.a(\a[5] ), .b(\b[4] ), .out0(new_n112));
  xorc02aa1n12x5               g017(.a(\a[8] ), .b(\b[7] ), .out0(new_n113));
  nor002aa1d32x5               g018(.a(\b[6] ), .b(\a[7] ), .o1(new_n114));
  nanp02aa1n12x5               g019(.a(\b[6] ), .b(\a[7] ), .o1(new_n115));
  norb02aa1n02x5               g020(.a(new_n115), .b(new_n114), .out0(new_n116));
  nanp02aa1n02x5               g021(.a(new_n113), .b(new_n116), .o1(new_n117));
  nano22aa1n03x7               g022(.a(new_n117), .b(new_n111), .c(new_n112), .out0(new_n118));
  tech160nm_finand02aa1n03p5x5 g023(.a(\b[5] ), .b(\a[6] ), .o1(new_n119));
  nano22aa1n03x7               g024(.a(new_n114), .b(new_n119), .c(new_n115), .out0(new_n120));
  oaih22aa1n04x5               g025(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n121));
  nanp03aa1n02x5               g026(.a(new_n120), .b(new_n113), .c(new_n121), .o1(new_n122));
  inv040aa1n02x5               g027(.a(new_n114), .o1(new_n123));
  oaoi03aa1n02x5               g028(.a(\a[8] ), .b(\b[7] ), .c(new_n123), .o1(new_n124));
  nanb02aa1n02x5               g029(.a(new_n124), .b(new_n122), .out0(new_n125));
  xorc02aa1n12x5               g030(.a(\a[9] ), .b(\b[8] ), .out0(new_n126));
  aoai13aa1n02x5               g031(.a(new_n126), .b(new_n125), .c(new_n110), .d(new_n118), .o1(new_n127));
  nor042aa1n04x5               g032(.a(\b[9] ), .b(\a[10] ), .o1(new_n128));
  nand42aa1n06x5               g033(.a(\b[9] ), .b(\a[10] ), .o1(new_n129));
  norb02aa1n06x5               g034(.a(new_n129), .b(new_n128), .out0(new_n130));
  xnbna2aa1n03x5               g035(.a(new_n130), .b(new_n127), .c(new_n98), .out0(\s[10] ));
  inv000aa1n04x5               g036(.a(new_n102), .o1(new_n132));
  nona22aa1n09x5               g037(.a(new_n132), .b(new_n107), .c(new_n103), .out0(new_n133));
  xnrc02aa1n02x5               g038(.a(\b[5] ), .b(\a[6] ), .out0(new_n134));
  nanb02aa1n12x5               g039(.a(new_n114), .b(new_n115), .out0(new_n135));
  nona23aa1n06x5               g040(.a(new_n112), .b(new_n113), .c(new_n134), .d(new_n135), .out0(new_n136));
  aoi013aa1n06x4               g041(.a(new_n124), .b(new_n120), .c(new_n113), .d(new_n121), .o1(new_n137));
  aoai13aa1n12x5               g042(.a(new_n137), .b(new_n136), .c(new_n133), .d(new_n109), .o1(new_n138));
  aoai13aa1n06x5               g043(.a(new_n130), .b(new_n97), .c(new_n138), .d(new_n126), .o1(new_n139));
  oaoi03aa1n02x5               g044(.a(\a[10] ), .b(\b[9] ), .c(new_n98), .o1(new_n140));
  inv000aa1d42x5               g045(.a(new_n140), .o1(new_n141));
  nor002aa1d32x5               g046(.a(\b[10] ), .b(\a[11] ), .o1(new_n142));
  nand42aa1n20x5               g047(.a(\b[10] ), .b(\a[11] ), .o1(new_n143));
  norb02aa1n02x5               g048(.a(new_n143), .b(new_n142), .out0(new_n144));
  xnbna2aa1n03x5               g049(.a(new_n144), .b(new_n139), .c(new_n141), .out0(\s[11] ));
  nand42aa1n03x5               g050(.a(new_n127), .b(new_n98), .o1(new_n146));
  aoai13aa1n02x5               g051(.a(new_n144), .b(new_n140), .c(new_n146), .d(new_n130), .o1(new_n147));
  nor002aa1n06x5               g052(.a(\b[11] ), .b(\a[12] ), .o1(new_n148));
  nand42aa1n20x5               g053(.a(\b[11] ), .b(\a[12] ), .o1(new_n149));
  norb02aa1n02x5               g054(.a(new_n149), .b(new_n148), .out0(new_n150));
  aoib12aa1n02x5               g055(.a(new_n142), .b(new_n149), .c(new_n148), .out0(new_n151));
  inv000aa1d42x5               g056(.a(new_n142), .o1(new_n152));
  inv000aa1d42x5               g057(.a(new_n144), .o1(new_n153));
  aoai13aa1n02x5               g058(.a(new_n152), .b(new_n153), .c(new_n139), .d(new_n141), .o1(new_n154));
  aoi022aa1n02x7               g059(.a(new_n154), .b(new_n150), .c(new_n147), .d(new_n151), .o1(\s[12] ));
  nano23aa1n09x5               g060(.a(new_n142), .b(new_n148), .c(new_n149), .d(new_n143), .out0(new_n156));
  nanp03aa1d24x5               g061(.a(new_n156), .b(new_n126), .c(new_n130), .o1(new_n157));
  inv000aa1d42x5               g062(.a(new_n157), .o1(new_n158));
  nano22aa1n03x7               g063(.a(new_n148), .b(new_n143), .c(new_n149), .out0(new_n159));
  tech160nm_fioai012aa1n04x5   g064(.a(new_n129), .b(\b[10] ), .c(\a[11] ), .o1(new_n160));
  oab012aa1n06x5               g065(.a(new_n160), .b(new_n97), .c(new_n128), .out0(new_n161));
  nanp02aa1n03x5               g066(.a(new_n161), .b(new_n159), .o1(new_n162));
  aoi012aa1n02x7               g067(.a(new_n148), .b(new_n142), .c(new_n149), .o1(new_n163));
  nand02aa1n02x5               g068(.a(new_n162), .b(new_n163), .o1(new_n164));
  nor002aa1n20x5               g069(.a(\b[12] ), .b(\a[13] ), .o1(new_n165));
  nand42aa1d28x5               g070(.a(\b[12] ), .b(\a[13] ), .o1(new_n166));
  norb02aa1n02x5               g071(.a(new_n166), .b(new_n165), .out0(new_n167));
  aoai13aa1n06x5               g072(.a(new_n167), .b(new_n164), .c(new_n138), .d(new_n158), .o1(new_n168));
  inv020aa1n02x5               g073(.a(new_n163), .o1(new_n169));
  aoi112aa1n02x5               g074(.a(new_n169), .b(new_n167), .c(new_n161), .d(new_n159), .o1(new_n170));
  aobi12aa1n02x5               g075(.a(new_n170), .b(new_n138), .c(new_n158), .out0(new_n171));
  norb02aa1n02x5               g076(.a(new_n168), .b(new_n171), .out0(\s[13] ));
  inv000aa1d42x5               g077(.a(new_n165), .o1(new_n173));
  nor042aa1n04x5               g078(.a(\b[13] ), .b(\a[14] ), .o1(new_n174));
  nand42aa1d28x5               g079(.a(\b[13] ), .b(\a[14] ), .o1(new_n175));
  norb02aa1n02x5               g080(.a(new_n175), .b(new_n174), .out0(new_n176));
  xnbna2aa1n03x5               g081(.a(new_n176), .b(new_n168), .c(new_n173), .out0(\s[14] ));
  nano23aa1d15x5               g082(.a(new_n165), .b(new_n174), .c(new_n175), .d(new_n166), .out0(new_n178));
  inv000aa1d42x5               g083(.a(new_n178), .o1(new_n179));
  nona22aa1n06x5               g084(.a(new_n138), .b(new_n157), .c(new_n179), .out0(new_n180));
  aoi012aa1n09x5               g085(.a(new_n174), .b(new_n165), .c(new_n175), .o1(new_n181));
  inv000aa1d42x5               g086(.a(new_n181), .o1(new_n182));
  tech160nm_fiaoi012aa1n05x5   g087(.a(new_n182), .b(new_n164), .c(new_n178), .o1(new_n183));
  xorc02aa1n12x5               g088(.a(\a[15] ), .b(\b[14] ), .out0(new_n184));
  aob012aa1n03x5               g089(.a(new_n184), .b(new_n180), .c(new_n183), .out0(new_n185));
  aoi112aa1n02x5               g090(.a(new_n184), .b(new_n182), .c(new_n164), .d(new_n178), .o1(new_n186));
  aobi12aa1n02x7               g091(.a(new_n185), .b(new_n186), .c(new_n180), .out0(\s[15] ));
  tech160nm_fixorc02aa1n04x5   g092(.a(\a[16] ), .b(\b[15] ), .out0(new_n188));
  oab012aa1n02x4               g093(.a(new_n188), .b(\a[15] ), .c(\b[14] ), .out0(new_n189));
  tech160nm_fioai012aa1n03p5x5 g094(.a(new_n185), .b(\b[14] ), .c(\a[15] ), .o1(new_n190));
  aoi022aa1n02x5               g095(.a(new_n190), .b(new_n188), .c(new_n185), .d(new_n189), .o1(\s[16] ));
  nano32aa1d15x5               g096(.a(new_n157), .b(new_n188), .c(new_n178), .d(new_n184), .out0(new_n192));
  aoai13aa1n06x5               g097(.a(new_n192), .b(new_n125), .c(new_n110), .d(new_n118), .o1(new_n193));
  nanp02aa1n02x5               g098(.a(new_n188), .b(new_n184), .o1(new_n194));
  aoi112aa1n02x5               g099(.a(\b[14] ), .b(\a[15] ), .c(\a[16] ), .d(\b[15] ), .o1(new_n195));
  oab012aa1n02x4               g100(.a(new_n195), .b(\a[16] ), .c(\b[15] ), .out0(new_n196));
  oai112aa1n06x5               g101(.a(new_n193), .b(new_n196), .c(new_n183), .d(new_n194), .o1(new_n197));
  xorc02aa1n12x5               g102(.a(\a[17] ), .b(\b[16] ), .out0(new_n198));
  aoai13aa1n06x5               g103(.a(new_n178), .b(new_n169), .c(new_n161), .d(new_n159), .o1(new_n199));
  aoi012aa1n02x5               g104(.a(new_n194), .b(new_n199), .c(new_n181), .o1(new_n200));
  norb03aa1n02x5               g105(.a(new_n196), .b(new_n200), .c(new_n198), .out0(new_n201));
  aoi022aa1n02x5               g106(.a(new_n197), .b(new_n198), .c(new_n201), .d(new_n193), .o1(\s[17] ));
  nor042aa1d18x5               g107(.a(\b[16] ), .b(\a[17] ), .o1(new_n203));
  inv000aa1d42x5               g108(.a(new_n203), .o1(new_n204));
  aoai13aa1n06x5               g109(.a(new_n196), .b(new_n194), .c(new_n199), .d(new_n181), .o1(new_n205));
  aoai13aa1n03x5               g110(.a(new_n198), .b(new_n205), .c(new_n138), .d(new_n192), .o1(new_n206));
  nor042aa1n09x5               g111(.a(\b[17] ), .b(\a[18] ), .o1(new_n207));
  nand02aa1n12x5               g112(.a(\b[17] ), .b(\a[18] ), .o1(new_n208));
  norb02aa1n06x4               g113(.a(new_n208), .b(new_n207), .out0(new_n209));
  xnbna2aa1n03x5               g114(.a(new_n209), .b(new_n206), .c(new_n204), .out0(\s[18] ));
  and002aa1n02x5               g115(.a(new_n198), .b(new_n209), .o(new_n211));
  aoai13aa1n06x5               g116(.a(new_n211), .b(new_n205), .c(new_n138), .d(new_n192), .o1(new_n212));
  oaoi03aa1n02x5               g117(.a(\a[18] ), .b(\b[17] ), .c(new_n204), .o1(new_n213));
  inv000aa1d42x5               g118(.a(new_n213), .o1(new_n214));
  nor002aa1d32x5               g119(.a(\b[18] ), .b(\a[19] ), .o1(new_n215));
  nand02aa1n06x5               g120(.a(\b[18] ), .b(\a[19] ), .o1(new_n216));
  norb02aa1n12x5               g121(.a(new_n216), .b(new_n215), .out0(new_n217));
  xnbna2aa1n03x5               g122(.a(new_n217), .b(new_n212), .c(new_n214), .out0(\s[19] ));
  xnrc02aa1n02x5               g123(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  aoai13aa1n03x5               g124(.a(new_n217), .b(new_n213), .c(new_n197), .d(new_n211), .o1(new_n220));
  nor042aa1n09x5               g125(.a(\b[19] ), .b(\a[20] ), .o1(new_n221));
  nand02aa1d28x5               g126(.a(\b[19] ), .b(\a[20] ), .o1(new_n222));
  norb02aa1n02x5               g127(.a(new_n222), .b(new_n221), .out0(new_n223));
  inv000aa1d42x5               g128(.a(\a[19] ), .o1(new_n224));
  inv000aa1d42x5               g129(.a(\b[18] ), .o1(new_n225));
  aboi22aa1n03x5               g130(.a(new_n221), .b(new_n222), .c(new_n224), .d(new_n225), .out0(new_n226));
  inv000aa1d42x5               g131(.a(new_n215), .o1(new_n227));
  inv000aa1d42x5               g132(.a(new_n217), .o1(new_n228));
  aoai13aa1n02x5               g133(.a(new_n227), .b(new_n228), .c(new_n212), .d(new_n214), .o1(new_n229));
  aoi022aa1n03x5               g134(.a(new_n229), .b(new_n223), .c(new_n220), .d(new_n226), .o1(\s[20] ));
  nano32aa1n03x7               g135(.a(new_n228), .b(new_n198), .c(new_n223), .d(new_n209), .out0(new_n231));
  aoai13aa1n03x5               g136(.a(new_n231), .b(new_n205), .c(new_n138), .d(new_n192), .o1(new_n232));
  nanb03aa1n12x5               g137(.a(new_n221), .b(new_n222), .c(new_n216), .out0(new_n233));
  oai112aa1n06x5               g138(.a(new_n227), .b(new_n208), .c(new_n207), .d(new_n203), .o1(new_n234));
  aoi012aa1d18x5               g139(.a(new_n221), .b(new_n215), .c(new_n222), .o1(new_n235));
  oai012aa1n18x5               g140(.a(new_n235), .b(new_n234), .c(new_n233), .o1(new_n236));
  inv000aa1d42x5               g141(.a(new_n236), .o1(new_n237));
  nanp02aa1n02x5               g142(.a(new_n232), .b(new_n237), .o1(new_n238));
  nor042aa1n12x5               g143(.a(\b[20] ), .b(\a[21] ), .o1(new_n239));
  nand42aa1n06x5               g144(.a(\b[20] ), .b(\a[21] ), .o1(new_n240));
  norb02aa1n03x5               g145(.a(new_n240), .b(new_n239), .out0(new_n241));
  nano22aa1n03x7               g146(.a(new_n221), .b(new_n216), .c(new_n222), .out0(new_n242));
  oai012aa1n04x7               g147(.a(new_n208), .b(\b[18] ), .c(\a[19] ), .o1(new_n243));
  oab012aa1n02x5               g148(.a(new_n243), .b(new_n203), .c(new_n207), .out0(new_n244));
  inv030aa1n03x5               g149(.a(new_n235), .o1(new_n245));
  aoi112aa1n02x5               g150(.a(new_n245), .b(new_n241), .c(new_n244), .d(new_n242), .o1(new_n246));
  aoi022aa1n02x5               g151(.a(new_n238), .b(new_n241), .c(new_n232), .d(new_n246), .o1(\s[21] ));
  aoai13aa1n03x5               g152(.a(new_n241), .b(new_n236), .c(new_n197), .d(new_n231), .o1(new_n248));
  nor042aa1n06x5               g153(.a(\b[21] ), .b(\a[22] ), .o1(new_n249));
  nand42aa1d28x5               g154(.a(\b[21] ), .b(\a[22] ), .o1(new_n250));
  norb02aa1n02x5               g155(.a(new_n250), .b(new_n249), .out0(new_n251));
  aoib12aa1n02x5               g156(.a(new_n239), .b(new_n250), .c(new_n249), .out0(new_n252));
  inv000aa1d42x5               g157(.a(new_n239), .o1(new_n253));
  inv000aa1d42x5               g158(.a(new_n241), .o1(new_n254));
  aoai13aa1n02x7               g159(.a(new_n253), .b(new_n254), .c(new_n232), .d(new_n237), .o1(new_n255));
  aoi022aa1n02x7               g160(.a(new_n255), .b(new_n251), .c(new_n248), .d(new_n252), .o1(\s[22] ));
  inv000aa1n02x5               g161(.a(new_n231), .o1(new_n257));
  nano22aa1n03x7               g162(.a(new_n257), .b(new_n241), .c(new_n251), .out0(new_n258));
  aoai13aa1n03x5               g163(.a(new_n258), .b(new_n205), .c(new_n138), .d(new_n192), .o1(new_n259));
  nano23aa1d15x5               g164(.a(new_n239), .b(new_n249), .c(new_n250), .d(new_n240), .out0(new_n260));
  aoi012aa1d24x5               g165(.a(new_n249), .b(new_n239), .c(new_n250), .o1(new_n261));
  inv000aa1d42x5               g166(.a(new_n261), .o1(new_n262));
  aoi012aa1n09x5               g167(.a(new_n262), .b(new_n236), .c(new_n260), .o1(new_n263));
  nanp02aa1n02x5               g168(.a(new_n259), .b(new_n263), .o1(new_n264));
  xorc02aa1n12x5               g169(.a(\a[23] ), .b(\b[22] ), .out0(new_n265));
  aoi112aa1n02x5               g170(.a(new_n265), .b(new_n262), .c(new_n236), .d(new_n260), .o1(new_n266));
  aoi022aa1n02x5               g171(.a(new_n264), .b(new_n265), .c(new_n259), .d(new_n266), .o1(\s[23] ));
  inv000aa1d42x5               g172(.a(new_n263), .o1(new_n268));
  aoai13aa1n03x5               g173(.a(new_n265), .b(new_n268), .c(new_n197), .d(new_n258), .o1(new_n269));
  tech160nm_fixorc02aa1n02p5x5 g174(.a(\a[24] ), .b(\b[23] ), .out0(new_n270));
  nor042aa1n06x5               g175(.a(\b[22] ), .b(\a[23] ), .o1(new_n271));
  norp02aa1n02x5               g176(.a(new_n270), .b(new_n271), .o1(new_n272));
  inv000aa1d42x5               g177(.a(new_n271), .o1(new_n273));
  inv000aa1d42x5               g178(.a(new_n265), .o1(new_n274));
  aoai13aa1n03x5               g179(.a(new_n273), .b(new_n274), .c(new_n259), .d(new_n263), .o1(new_n275));
  aoi022aa1n03x5               g180(.a(new_n275), .b(new_n270), .c(new_n269), .d(new_n272), .o1(\s[24] ));
  and002aa1n06x5               g181(.a(new_n270), .b(new_n265), .o(new_n277));
  nano22aa1n02x4               g182(.a(new_n257), .b(new_n277), .c(new_n260), .out0(new_n278));
  aoai13aa1n03x5               g183(.a(new_n278), .b(new_n205), .c(new_n138), .d(new_n192), .o1(new_n279));
  aoai13aa1n04x5               g184(.a(new_n260), .b(new_n245), .c(new_n244), .d(new_n242), .o1(new_n280));
  inv000aa1n06x5               g185(.a(new_n277), .o1(new_n281));
  oao003aa1n03x5               g186(.a(\a[24] ), .b(\b[23] ), .c(new_n273), .carry(new_n282));
  aoai13aa1n12x5               g187(.a(new_n282), .b(new_n281), .c(new_n280), .d(new_n261), .o1(new_n283));
  inv000aa1d42x5               g188(.a(new_n283), .o1(new_n284));
  nanp02aa1n02x5               g189(.a(new_n279), .b(new_n284), .o1(new_n285));
  xorc02aa1n12x5               g190(.a(\a[25] ), .b(\b[24] ), .out0(new_n286));
  aoai13aa1n04x5               g191(.a(new_n277), .b(new_n262), .c(new_n236), .d(new_n260), .o1(new_n287));
  inv000aa1d42x5               g192(.a(new_n286), .o1(new_n288));
  and003aa1n02x5               g193(.a(new_n287), .b(new_n288), .c(new_n282), .o(new_n289));
  aoi022aa1n02x5               g194(.a(new_n285), .b(new_n286), .c(new_n279), .d(new_n289), .o1(\s[25] ));
  aoai13aa1n03x5               g195(.a(new_n286), .b(new_n283), .c(new_n197), .d(new_n278), .o1(new_n291));
  tech160nm_fixorc02aa1n03p5x5 g196(.a(\a[26] ), .b(\b[25] ), .out0(new_n292));
  nor042aa1n03x5               g197(.a(\b[24] ), .b(\a[25] ), .o1(new_n293));
  norp02aa1n02x5               g198(.a(new_n292), .b(new_n293), .o1(new_n294));
  inv000aa1d42x5               g199(.a(new_n293), .o1(new_n295));
  aoai13aa1n03x5               g200(.a(new_n295), .b(new_n288), .c(new_n279), .d(new_n284), .o1(new_n296));
  aoi022aa1n02x7               g201(.a(new_n296), .b(new_n292), .c(new_n291), .d(new_n294), .o1(\s[26] ));
  and002aa1n12x5               g202(.a(new_n292), .b(new_n286), .o(new_n298));
  nano32aa1n03x7               g203(.a(new_n257), .b(new_n298), .c(new_n260), .d(new_n277), .out0(new_n299));
  aoai13aa1n06x5               g204(.a(new_n299), .b(new_n205), .c(new_n138), .d(new_n192), .o1(new_n300));
  oao003aa1n02x5               g205(.a(\a[26] ), .b(\b[25] ), .c(new_n295), .carry(new_n301));
  inv000aa1d42x5               g206(.a(new_n301), .o1(new_n302));
  aoi012aa1n12x5               g207(.a(new_n302), .b(new_n283), .c(new_n298), .o1(new_n303));
  nanp02aa1n03x5               g208(.a(new_n300), .b(new_n303), .o1(new_n304));
  xorc02aa1n12x5               g209(.a(\a[27] ), .b(\b[26] ), .out0(new_n305));
  aoi112aa1n02x5               g210(.a(new_n305), .b(new_n302), .c(new_n283), .d(new_n298), .o1(new_n306));
  aoi022aa1n02x5               g211(.a(new_n304), .b(new_n305), .c(new_n300), .d(new_n306), .o1(\s[27] ));
  inv000aa1d42x5               g212(.a(new_n298), .o1(new_n308));
  aoai13aa1n04x5               g213(.a(new_n301), .b(new_n308), .c(new_n287), .d(new_n282), .o1(new_n309));
  aoai13aa1n02x5               g214(.a(new_n305), .b(new_n309), .c(new_n197), .d(new_n299), .o1(new_n310));
  xorc02aa1n02x5               g215(.a(\a[28] ), .b(\b[27] ), .out0(new_n311));
  norp02aa1n02x5               g216(.a(\b[26] ), .b(\a[27] ), .o1(new_n312));
  norp02aa1n02x5               g217(.a(new_n311), .b(new_n312), .o1(new_n313));
  inv000aa1n03x5               g218(.a(new_n312), .o1(new_n314));
  inv000aa1d42x5               g219(.a(new_n305), .o1(new_n315));
  aoai13aa1n03x5               g220(.a(new_n314), .b(new_n315), .c(new_n300), .d(new_n303), .o1(new_n316));
  aoi022aa1n03x5               g221(.a(new_n316), .b(new_n311), .c(new_n310), .d(new_n313), .o1(\s[28] ));
  and002aa1n02x5               g222(.a(new_n311), .b(new_n305), .o(new_n318));
  aoai13aa1n03x5               g223(.a(new_n318), .b(new_n309), .c(new_n197), .d(new_n299), .o1(new_n319));
  xorc02aa1n02x5               g224(.a(\a[29] ), .b(\b[28] ), .out0(new_n320));
  oao003aa1n02x5               g225(.a(\a[28] ), .b(\b[27] ), .c(new_n314), .carry(new_n321));
  norb02aa1n02x5               g226(.a(new_n321), .b(new_n320), .out0(new_n322));
  inv000aa1d42x5               g227(.a(new_n318), .o1(new_n323));
  aoai13aa1n04x5               g228(.a(new_n321), .b(new_n323), .c(new_n300), .d(new_n303), .o1(new_n324));
  aoi022aa1n02x7               g229(.a(new_n324), .b(new_n320), .c(new_n319), .d(new_n322), .o1(\s[29] ));
  xorb03aa1n02x5               g230(.a(new_n101), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n06x5               g231(.a(new_n315), .b(new_n311), .c(new_n320), .out0(new_n327));
  aoai13aa1n02x5               g232(.a(new_n327), .b(new_n309), .c(new_n197), .d(new_n299), .o1(new_n328));
  xorc02aa1n02x5               g233(.a(\a[30] ), .b(\b[29] ), .out0(new_n329));
  oao003aa1n02x5               g234(.a(\a[29] ), .b(\b[28] ), .c(new_n321), .carry(new_n330));
  norb02aa1n02x5               g235(.a(new_n330), .b(new_n329), .out0(new_n331));
  inv000aa1d42x5               g236(.a(new_n327), .o1(new_n332));
  aoai13aa1n03x5               g237(.a(new_n330), .b(new_n332), .c(new_n300), .d(new_n303), .o1(new_n333));
  aoi022aa1n03x5               g238(.a(new_n333), .b(new_n329), .c(new_n328), .d(new_n331), .o1(\s[30] ));
  nano32aa1n06x5               g239(.a(new_n315), .b(new_n329), .c(new_n311), .d(new_n320), .out0(new_n335));
  aoai13aa1n02x5               g240(.a(new_n335), .b(new_n309), .c(new_n197), .d(new_n299), .o1(new_n336));
  xorc02aa1n02x5               g241(.a(\a[31] ), .b(\b[30] ), .out0(new_n337));
  oao003aa1n02x5               g242(.a(\a[30] ), .b(\b[29] ), .c(new_n330), .carry(new_n338));
  norb02aa1n02x5               g243(.a(new_n338), .b(new_n337), .out0(new_n339));
  inv000aa1d42x5               g244(.a(new_n335), .o1(new_n340));
  aoai13aa1n03x5               g245(.a(new_n338), .b(new_n340), .c(new_n300), .d(new_n303), .o1(new_n341));
  aoi022aa1n03x5               g246(.a(new_n341), .b(new_n337), .c(new_n336), .d(new_n339), .o1(\s[31] ));
  norb02aa1n02x5               g247(.a(new_n104), .b(new_n106), .out0(new_n343));
  xobna2aa1n03x5               g248(.a(new_n343), .b(new_n132), .c(new_n100), .out0(\s[3] ));
  aoai13aa1n02x5               g249(.a(new_n343), .b(new_n102), .c(\a[2] ), .d(\b[1] ), .o1(new_n345));
  nanb02aa1n02x5               g250(.a(new_n108), .b(new_n105), .out0(new_n346));
  xnbna2aa1n03x5               g251(.a(new_n346), .b(new_n345), .c(new_n104), .out0(\s[4] ));
  xnbna2aa1n03x5               g252(.a(new_n112), .b(new_n133), .c(new_n109), .out0(\s[5] ));
  inv000aa1d42x5               g253(.a(\a[5] ), .o1(new_n349));
  inv000aa1d42x5               g254(.a(\b[4] ), .o1(new_n350));
  aobi12aa1n03x5               g255(.a(new_n112), .b(new_n133), .c(new_n109), .out0(new_n351));
  aoai13aa1n06x5               g256(.a(new_n111), .b(new_n351), .c(new_n350), .d(new_n349), .o1(new_n352));
  aoi112aa1n02x5               g257(.a(new_n351), .b(new_n111), .c(new_n349), .d(new_n350), .o1(new_n353));
  norb02aa1n02x5               g258(.a(new_n352), .b(new_n353), .out0(\s[6] ));
  nanp02aa1n02x5               g259(.a(new_n121), .b(new_n119), .o1(new_n355));
  xnbna2aa1n03x5               g260(.a(new_n116), .b(new_n352), .c(new_n355), .out0(\s[7] ));
  aob012aa1n03x5               g261(.a(new_n116), .b(new_n352), .c(new_n355), .out0(new_n357));
  aoai13aa1n02x5               g262(.a(new_n123), .b(new_n135), .c(new_n352), .d(new_n355), .o1(new_n358));
  norp02aa1n02x5               g263(.a(new_n113), .b(new_n114), .o1(new_n359));
  aoi022aa1n02x5               g264(.a(new_n358), .b(new_n113), .c(new_n357), .d(new_n359), .o1(\s[8] ));
  nanp02aa1n02x5               g265(.a(new_n110), .b(new_n118), .o1(new_n361));
  aoi113aa1n02x5               g266(.a(new_n126), .b(new_n124), .c(new_n120), .d(new_n113), .e(new_n121), .o1(new_n362));
  aoi022aa1n02x5               g267(.a(new_n138), .b(new_n126), .c(new_n361), .d(new_n362), .o1(\s[9] ));
endmodule


