// Benchmark "adder" written by ABC on Thu Jul 18 10:22:26 2024

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
    new_n125, new_n126, new_n127, new_n129, new_n130, new_n131, new_n132,
    new_n133, new_n134, new_n135, new_n136, new_n138, new_n139, new_n140,
    new_n141, new_n142, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n157, new_n158, new_n159, new_n160, new_n161, new_n163, new_n164,
    new_n165, new_n166, new_n167, new_n168, new_n170, new_n171, new_n172,
    new_n173, new_n174, new_n175, new_n176, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n187, new_n188,
    new_n189, new_n190, new_n191, new_n192, new_n194, new_n195, new_n196,
    new_n197, new_n198, new_n199, new_n200, new_n201, new_n202, new_n205,
    new_n206, new_n207, new_n208, new_n209, new_n210, new_n211, new_n212,
    new_n213, new_n215, new_n216, new_n217, new_n218, new_n219, new_n220,
    new_n221, new_n222, new_n224, new_n225, new_n226, new_n227, new_n228,
    new_n229, new_n230, new_n232, new_n233, new_n234, new_n235, new_n236,
    new_n237, new_n238, new_n239, new_n241, new_n242, new_n243, new_n244,
    new_n245, new_n246, new_n247, new_n248, new_n250, new_n251, new_n252,
    new_n253, new_n254, new_n255, new_n256, new_n257, new_n258, new_n259,
    new_n261, new_n262, new_n263, new_n264, new_n265, new_n266, new_n267,
    new_n269, new_n270, new_n271, new_n272, new_n273, new_n274, new_n275,
    new_n276, new_n278, new_n279, new_n280, new_n281, new_n282, new_n283,
    new_n284, new_n285, new_n286, new_n288, new_n289, new_n290, new_n291,
    new_n292, new_n293, new_n294, new_n297, new_n298, new_n299, new_n300,
    new_n301, new_n302, new_n303, new_n305, new_n306, new_n307, new_n308,
    new_n309, new_n310, new_n311, new_n312, new_n313, new_n315, new_n317,
    new_n318, new_n321, new_n322, new_n324, new_n326;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  norp02aa1n02x5               g001(.a(\b[9] ), .b(\a[10] ), .o1(new_n97));
  nand22aa1n12x5               g002(.a(\b[9] ), .b(\a[10] ), .o1(new_n98));
  nanb02aa1n06x5               g003(.a(new_n97), .b(new_n98), .out0(new_n99));
  nor042aa1n02x5               g004(.a(\b[8] ), .b(\a[9] ), .o1(new_n100));
  inv000aa1d42x5               g005(.a(\a[2] ), .o1(new_n101));
  inv000aa1d42x5               g006(.a(\b[1] ), .o1(new_n102));
  nanp02aa1n02x5               g007(.a(\b[0] ), .b(\a[1] ), .o1(new_n103));
  oao003aa1n02x5               g008(.a(new_n101), .b(new_n102), .c(new_n103), .carry(new_n104));
  xorc02aa1n02x5               g009(.a(\a[4] ), .b(\b[3] ), .out0(new_n105));
  xorc02aa1n02x5               g010(.a(\a[3] ), .b(\b[2] ), .out0(new_n106));
  nanp03aa1n02x5               g011(.a(new_n104), .b(new_n105), .c(new_n106), .o1(new_n107));
  orn002aa1n02x5               g012(.a(\a[4] ), .b(\b[3] ), .o(new_n108));
  aoi112aa1n02x5               g013(.a(\b[2] ), .b(\a[3] ), .c(\a[4] ), .d(\b[3] ), .o1(new_n109));
  norb02aa1n02x5               g014(.a(new_n108), .b(new_n109), .out0(new_n110));
  xnrc02aa1n02x5               g015(.a(\b[5] ), .b(\a[6] ), .out0(new_n111));
  tech160nm_fixorc02aa1n03p5x5 g016(.a(\a[5] ), .b(\b[4] ), .out0(new_n112));
  xorc02aa1n12x5               g017(.a(\a[8] ), .b(\b[7] ), .out0(new_n113));
  nor042aa1d18x5               g018(.a(\b[6] ), .b(\a[7] ), .o1(new_n114));
  nand42aa1n02x5               g019(.a(\b[6] ), .b(\a[7] ), .o1(new_n115));
  nanb02aa1n06x5               g020(.a(new_n114), .b(new_n115), .out0(new_n116));
  nona23aa1n06x5               g021(.a(new_n112), .b(new_n113), .c(new_n111), .d(new_n116), .out0(new_n117));
  nanp02aa1n02x5               g022(.a(\b[5] ), .b(\a[6] ), .o1(new_n118));
  nano22aa1n03x7               g023(.a(new_n114), .b(new_n118), .c(new_n115), .out0(new_n119));
  oai022aa1n06x5               g024(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n120));
  inv000aa1d42x5               g025(.a(new_n114), .o1(new_n121));
  oaoi03aa1n02x5               g026(.a(\a[8] ), .b(\b[7] ), .c(new_n121), .o1(new_n122));
  aoi013aa1n09x5               g027(.a(new_n122), .b(new_n119), .c(new_n113), .d(new_n120), .o1(new_n123));
  aoai13aa1n12x5               g028(.a(new_n123), .b(new_n117), .c(new_n107), .d(new_n110), .o1(new_n124));
  xorc02aa1n12x5               g029(.a(\a[9] ), .b(\b[8] ), .out0(new_n125));
  aoai13aa1n02x5               g030(.a(new_n99), .b(new_n100), .c(new_n124), .d(new_n125), .o1(new_n126));
  aoi112aa1n06x5               g031(.a(new_n99), .b(new_n100), .c(new_n124), .d(new_n125), .o1(new_n127));
  nanb02aa1n02x5               g032(.a(new_n127), .b(new_n126), .out0(\s[10] ));
  inv000aa1d42x5               g033(.a(new_n98), .o1(new_n129));
  nanp02aa1n02x5               g034(.a(\b[10] ), .b(\a[11] ), .o1(new_n130));
  inv040aa1d32x5               g035(.a(\a[11] ), .o1(new_n131));
  inv000aa1d42x5               g036(.a(\b[10] ), .o1(new_n132));
  nand42aa1n02x5               g037(.a(new_n132), .b(new_n131), .o1(new_n133));
  and003aa1n02x5               g038(.a(new_n133), .b(new_n130), .c(new_n98), .o(new_n134));
  norb02aa1n02x5               g039(.a(new_n134), .b(new_n127), .out0(new_n135));
  nanp02aa1n02x5               g040(.a(new_n133), .b(new_n130), .o1(new_n136));
  oaoi13aa1n02x5               g041(.a(new_n135), .b(new_n136), .c(new_n129), .d(new_n127), .o1(\s[11] ));
  tech160nm_fixorc02aa1n03p5x5 g042(.a(\a[12] ), .b(\b[11] ), .out0(new_n138));
  orn002aa1n02x5               g043(.a(\a[12] ), .b(\b[11] ), .o(new_n139));
  and002aa1n02x5               g044(.a(\b[11] ), .b(\a[12] ), .o(new_n140));
  aboi22aa1n03x5               g045(.a(new_n140), .b(new_n139), .c(new_n132), .d(new_n131), .out0(new_n141));
  oaib12aa1n02x5               g046(.a(new_n133), .b(new_n127), .c(new_n134), .out0(new_n142));
  aboi22aa1n03x5               g047(.a(new_n135), .b(new_n141), .c(new_n142), .d(new_n138), .out0(\s[12] ));
  oaoi03aa1n02x5               g048(.a(new_n101), .b(new_n102), .c(new_n103), .o1(new_n144));
  xnrc02aa1n02x5               g049(.a(\b[3] ), .b(\a[4] ), .out0(new_n145));
  xnrc02aa1n02x5               g050(.a(\b[2] ), .b(\a[3] ), .out0(new_n146));
  oai013aa1n02x4               g051(.a(new_n110), .b(new_n144), .c(new_n145), .d(new_n146), .o1(new_n147));
  nano23aa1n02x4               g052(.a(new_n116), .b(new_n111), .c(new_n112), .d(new_n113), .out0(new_n148));
  inv000aa1d42x5               g053(.a(new_n123), .o1(new_n149));
  nona23aa1d18x5               g054(.a(new_n138), .b(new_n125), .c(new_n99), .d(new_n136), .out0(new_n150));
  inv000aa1d42x5               g055(.a(new_n150), .o1(new_n151));
  aoai13aa1n02x5               g056(.a(new_n151), .b(new_n149), .c(new_n148), .d(new_n147), .o1(new_n152));
  aoai13aa1n02x5               g057(.a(new_n130), .b(new_n97), .c(new_n100), .d(new_n98), .o1(new_n153));
  aoai13aa1n03x5               g058(.a(new_n139), .b(new_n140), .c(new_n153), .d(new_n133), .o1(new_n154));
  nanb02aa1n02x5               g059(.a(new_n154), .b(new_n152), .out0(new_n155));
  xorb03aa1n02x5               g060(.a(new_n155), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  inv000aa1d42x5               g061(.a(\a[13] ), .o1(new_n157));
  nanb02aa1n12x5               g062(.a(\b[12] ), .b(new_n157), .out0(new_n158));
  xorc02aa1n02x5               g063(.a(\a[13] ), .b(\b[12] ), .out0(new_n159));
  aoai13aa1n02x5               g064(.a(new_n159), .b(new_n154), .c(new_n124), .d(new_n151), .o1(new_n160));
  xorc02aa1n02x5               g065(.a(\a[14] ), .b(\b[13] ), .out0(new_n161));
  xnbna2aa1n03x5               g066(.a(new_n161), .b(new_n160), .c(new_n158), .out0(\s[14] ));
  inv000aa1d42x5               g067(.a(\a[14] ), .o1(new_n163));
  xroi22aa1d04x5               g068(.a(new_n157), .b(\b[12] ), .c(new_n163), .d(\b[13] ), .out0(new_n164));
  aoai13aa1n02x5               g069(.a(new_n164), .b(new_n154), .c(new_n124), .d(new_n151), .o1(new_n165));
  oaoi03aa1n12x5               g070(.a(\a[14] ), .b(\b[13] ), .c(new_n158), .o1(new_n166));
  inv000aa1d42x5               g071(.a(new_n166), .o1(new_n167));
  xorc02aa1n12x5               g072(.a(\a[15] ), .b(\b[14] ), .out0(new_n168));
  xnbna2aa1n03x5               g073(.a(new_n168), .b(new_n165), .c(new_n167), .out0(\s[15] ));
  aoai13aa1n02x5               g074(.a(new_n168), .b(new_n166), .c(new_n155), .d(new_n164), .o1(new_n170));
  xorc02aa1n02x5               g075(.a(\a[16] ), .b(\b[15] ), .out0(new_n171));
  nor042aa1n03x5               g076(.a(\b[14] ), .b(\a[15] ), .o1(new_n172));
  norp02aa1n02x5               g077(.a(new_n171), .b(new_n172), .o1(new_n173));
  inv000aa1d42x5               g078(.a(new_n172), .o1(new_n174));
  inv000aa1d42x5               g079(.a(new_n168), .o1(new_n175));
  aoai13aa1n02x5               g080(.a(new_n174), .b(new_n175), .c(new_n165), .d(new_n167), .o1(new_n176));
  aoi022aa1n02x5               g081(.a(new_n176), .b(new_n171), .c(new_n170), .d(new_n173), .o1(\s[16] ));
  nand22aa1n03x5               g082(.a(new_n171), .b(new_n168), .o1(new_n178));
  norb03aa1n12x5               g083(.a(new_n164), .b(new_n150), .c(new_n178), .out0(new_n179));
  nanp02aa1n09x5               g084(.a(new_n124), .b(new_n179), .o1(new_n180));
  nano22aa1n02x4               g085(.a(new_n178), .b(new_n159), .c(new_n161), .out0(new_n181));
  oaoi03aa1n02x5               g086(.a(\a[16] ), .b(\b[15] ), .c(new_n174), .o1(new_n182));
  aoi013aa1n03x5               g087(.a(new_n182), .b(new_n166), .c(new_n168), .d(new_n171), .o1(new_n183));
  aobi12aa1n06x5               g088(.a(new_n183), .b(new_n181), .c(new_n154), .out0(new_n184));
  xorc02aa1n02x5               g089(.a(\a[17] ), .b(\b[16] ), .out0(new_n185));
  xnbna2aa1n03x5               g090(.a(new_n185), .b(new_n180), .c(new_n184), .out0(\s[17] ));
  inv000aa1d42x5               g091(.a(\a[17] ), .o1(new_n187));
  nanb02aa1n02x5               g092(.a(\b[16] ), .b(new_n187), .out0(new_n188));
  nanp02aa1n02x5               g093(.a(new_n181), .b(new_n154), .o1(new_n189));
  nanp02aa1n09x5               g094(.a(new_n189), .b(new_n183), .o1(new_n190));
  aoai13aa1n02x5               g095(.a(new_n185), .b(new_n190), .c(new_n124), .d(new_n179), .o1(new_n191));
  xorc02aa1n02x5               g096(.a(\a[18] ), .b(\b[17] ), .out0(new_n192));
  xnbna2aa1n03x5               g097(.a(new_n192), .b(new_n191), .c(new_n188), .out0(\s[18] ));
  inv000aa1d42x5               g098(.a(\a[18] ), .o1(new_n194));
  xroi22aa1d04x5               g099(.a(new_n187), .b(\b[16] ), .c(new_n194), .d(\b[17] ), .out0(new_n195));
  aoai13aa1n02x5               g100(.a(new_n195), .b(new_n190), .c(new_n124), .d(new_n179), .o1(new_n196));
  oai022aa1n02x5               g101(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n197));
  oaib12aa1n06x5               g102(.a(new_n197), .b(new_n194), .c(\b[17] ), .out0(new_n198));
  nor042aa1n06x5               g103(.a(\b[18] ), .b(\a[19] ), .o1(new_n199));
  nanp02aa1n02x5               g104(.a(\b[18] ), .b(\a[19] ), .o1(new_n200));
  nanb02aa1n02x5               g105(.a(new_n199), .b(new_n200), .out0(new_n201));
  inv000aa1d42x5               g106(.a(new_n201), .o1(new_n202));
  xnbna2aa1n03x5               g107(.a(new_n202), .b(new_n196), .c(new_n198), .out0(\s[19] ));
  xnrc02aa1n02x5               g108(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nand22aa1n04x5               g109(.a(new_n180), .b(new_n184), .o1(new_n205));
  oaoi03aa1n02x5               g110(.a(\a[18] ), .b(\b[17] ), .c(new_n188), .o1(new_n206));
  aoai13aa1n02x5               g111(.a(new_n202), .b(new_n206), .c(new_n205), .d(new_n195), .o1(new_n207));
  norp02aa1n04x5               g112(.a(\b[19] ), .b(\a[20] ), .o1(new_n208));
  nand02aa1n04x5               g113(.a(\b[19] ), .b(\a[20] ), .o1(new_n209));
  norb02aa1n02x5               g114(.a(new_n209), .b(new_n208), .out0(new_n210));
  aoib12aa1n02x5               g115(.a(new_n199), .b(new_n209), .c(new_n208), .out0(new_n211));
  inv000aa1d42x5               g116(.a(new_n199), .o1(new_n212));
  aoai13aa1n02x5               g117(.a(new_n212), .b(new_n201), .c(new_n196), .d(new_n198), .o1(new_n213));
  aoi022aa1n02x5               g118(.a(new_n213), .b(new_n210), .c(new_n207), .d(new_n211), .o1(\s[20] ));
  nona23aa1n09x5               g119(.a(new_n209), .b(new_n200), .c(new_n199), .d(new_n208), .out0(new_n215));
  nano22aa1n03x7               g120(.a(new_n215), .b(new_n185), .c(new_n192), .out0(new_n216));
  aoai13aa1n02x5               g121(.a(new_n216), .b(new_n190), .c(new_n124), .d(new_n179), .o1(new_n217));
  aoi012aa1n12x5               g122(.a(new_n208), .b(new_n199), .c(new_n209), .o1(new_n218));
  oai012aa1d24x5               g123(.a(new_n218), .b(new_n215), .c(new_n198), .o1(new_n219));
  inv000aa1d42x5               g124(.a(new_n219), .o1(new_n220));
  xnrc02aa1n12x5               g125(.a(\b[20] ), .b(\a[21] ), .out0(new_n221));
  inv000aa1d42x5               g126(.a(new_n221), .o1(new_n222));
  xnbna2aa1n03x5               g127(.a(new_n222), .b(new_n217), .c(new_n220), .out0(\s[21] ));
  aoai13aa1n02x5               g128(.a(new_n222), .b(new_n219), .c(new_n205), .d(new_n216), .o1(new_n224));
  xnrc02aa1n12x5               g129(.a(\b[21] ), .b(\a[22] ), .out0(new_n225));
  inv000aa1d42x5               g130(.a(new_n225), .o1(new_n226));
  nor042aa1n06x5               g131(.a(\b[20] ), .b(\a[21] ), .o1(new_n227));
  norb02aa1n02x5               g132(.a(new_n225), .b(new_n227), .out0(new_n228));
  inv000aa1d42x5               g133(.a(new_n227), .o1(new_n229));
  aoai13aa1n02x5               g134(.a(new_n229), .b(new_n221), .c(new_n217), .d(new_n220), .o1(new_n230));
  aoi022aa1n02x5               g135(.a(new_n230), .b(new_n226), .c(new_n224), .d(new_n228), .o1(\s[22] ));
  nano23aa1n06x5               g136(.a(new_n199), .b(new_n208), .c(new_n209), .d(new_n200), .out0(new_n232));
  nor042aa1n06x5               g137(.a(new_n225), .b(new_n221), .o1(new_n233));
  and003aa1n02x5               g138(.a(new_n195), .b(new_n233), .c(new_n232), .o(new_n234));
  aoai13aa1n02x5               g139(.a(new_n234), .b(new_n190), .c(new_n124), .d(new_n179), .o1(new_n235));
  oao003aa1n12x5               g140(.a(\a[22] ), .b(\b[21] ), .c(new_n229), .carry(new_n236));
  inv000aa1d42x5               g141(.a(new_n236), .o1(new_n237));
  aoi012aa1d24x5               g142(.a(new_n237), .b(new_n219), .c(new_n233), .o1(new_n238));
  xorc02aa1n12x5               g143(.a(\a[23] ), .b(\b[22] ), .out0(new_n239));
  xnbna2aa1n03x5               g144(.a(new_n239), .b(new_n235), .c(new_n238), .out0(\s[23] ));
  inv000aa1d42x5               g145(.a(new_n238), .o1(new_n241));
  aoai13aa1n02x5               g146(.a(new_n239), .b(new_n241), .c(new_n205), .d(new_n234), .o1(new_n242));
  xorc02aa1n02x5               g147(.a(\a[24] ), .b(\b[23] ), .out0(new_n243));
  nor042aa1n03x5               g148(.a(\b[22] ), .b(\a[23] ), .o1(new_n244));
  norp02aa1n02x5               g149(.a(new_n243), .b(new_n244), .o1(new_n245));
  inv000aa1d42x5               g150(.a(new_n244), .o1(new_n246));
  inv000aa1d42x5               g151(.a(new_n239), .o1(new_n247));
  aoai13aa1n02x5               g152(.a(new_n246), .b(new_n247), .c(new_n235), .d(new_n238), .o1(new_n248));
  aoi022aa1n02x5               g153(.a(new_n248), .b(new_n243), .c(new_n242), .d(new_n245), .o1(\s[24] ));
  and002aa1n06x5               g154(.a(new_n243), .b(new_n239), .o(new_n250));
  inv000aa1d42x5               g155(.a(new_n250), .o1(new_n251));
  nano32aa1n02x4               g156(.a(new_n251), .b(new_n195), .c(new_n233), .d(new_n232), .out0(new_n252));
  aoai13aa1n02x5               g157(.a(new_n252), .b(new_n190), .c(new_n124), .d(new_n179), .o1(new_n253));
  inv040aa1n02x5               g158(.a(new_n218), .o1(new_n254));
  aoai13aa1n06x5               g159(.a(new_n233), .b(new_n254), .c(new_n232), .d(new_n206), .o1(new_n255));
  oao003aa1n02x5               g160(.a(\a[24] ), .b(\b[23] ), .c(new_n246), .carry(new_n256));
  aoai13aa1n12x5               g161(.a(new_n256), .b(new_n251), .c(new_n255), .d(new_n236), .o1(new_n257));
  inv000aa1d42x5               g162(.a(new_n257), .o1(new_n258));
  xorc02aa1n12x5               g163(.a(\a[25] ), .b(\b[24] ), .out0(new_n259));
  xnbna2aa1n03x5               g164(.a(new_n259), .b(new_n253), .c(new_n258), .out0(\s[25] ));
  aoai13aa1n02x5               g165(.a(new_n259), .b(new_n257), .c(new_n205), .d(new_n252), .o1(new_n261));
  xorc02aa1n02x5               g166(.a(\a[26] ), .b(\b[25] ), .out0(new_n262));
  nor042aa1n03x5               g167(.a(\b[24] ), .b(\a[25] ), .o1(new_n263));
  norp02aa1n02x5               g168(.a(new_n262), .b(new_n263), .o1(new_n264));
  inv000aa1d42x5               g169(.a(new_n263), .o1(new_n265));
  inv000aa1d42x5               g170(.a(new_n259), .o1(new_n266));
  aoai13aa1n02x5               g171(.a(new_n265), .b(new_n266), .c(new_n253), .d(new_n258), .o1(new_n267));
  aoi022aa1n02x5               g172(.a(new_n267), .b(new_n262), .c(new_n261), .d(new_n264), .o1(\s[26] ));
  and002aa1n02x5               g173(.a(new_n262), .b(new_n259), .o(new_n269));
  inv000aa1n02x5               g174(.a(new_n269), .o1(new_n270));
  nano32aa1n03x7               g175(.a(new_n270), .b(new_n216), .c(new_n233), .d(new_n250), .out0(new_n271));
  aoai13aa1n06x5               g176(.a(new_n271), .b(new_n190), .c(new_n124), .d(new_n179), .o1(new_n272));
  oao003aa1n02x5               g177(.a(\a[26] ), .b(\b[25] ), .c(new_n265), .carry(new_n273));
  inv000aa1d42x5               g178(.a(new_n273), .o1(new_n274));
  aoi012aa1n12x5               g179(.a(new_n274), .b(new_n257), .c(new_n269), .o1(new_n275));
  xorc02aa1n12x5               g180(.a(\a[27] ), .b(\b[26] ), .out0(new_n276));
  xnbna2aa1n03x5               g181(.a(new_n276), .b(new_n275), .c(new_n272), .out0(\s[27] ));
  aoai13aa1n04x5               g182(.a(new_n250), .b(new_n237), .c(new_n219), .d(new_n233), .o1(new_n278));
  aoai13aa1n06x5               g183(.a(new_n273), .b(new_n270), .c(new_n278), .d(new_n256), .o1(new_n279));
  aoai13aa1n02x5               g184(.a(new_n276), .b(new_n279), .c(new_n205), .d(new_n271), .o1(new_n280));
  tech160nm_fixorc02aa1n02p5x5 g185(.a(\a[28] ), .b(\b[27] ), .out0(new_n281));
  norp02aa1n02x5               g186(.a(\b[26] ), .b(\a[27] ), .o1(new_n282));
  norp02aa1n02x5               g187(.a(new_n281), .b(new_n282), .o1(new_n283));
  inv000aa1n03x5               g188(.a(new_n282), .o1(new_n284));
  inv000aa1d42x5               g189(.a(new_n276), .o1(new_n285));
  aoai13aa1n02x5               g190(.a(new_n284), .b(new_n285), .c(new_n275), .d(new_n272), .o1(new_n286));
  aoi022aa1n02x5               g191(.a(new_n286), .b(new_n281), .c(new_n280), .d(new_n283), .o1(\s[28] ));
  and002aa1n02x5               g192(.a(new_n281), .b(new_n276), .o(new_n288));
  aoai13aa1n02x5               g193(.a(new_n288), .b(new_n279), .c(new_n205), .d(new_n271), .o1(new_n289));
  inv000aa1d42x5               g194(.a(new_n288), .o1(new_n290));
  oao003aa1n02x5               g195(.a(\a[28] ), .b(\b[27] ), .c(new_n284), .carry(new_n291));
  aoai13aa1n02x5               g196(.a(new_n291), .b(new_n290), .c(new_n275), .d(new_n272), .o1(new_n292));
  xorc02aa1n02x5               g197(.a(\a[29] ), .b(\b[28] ), .out0(new_n293));
  norb02aa1n02x5               g198(.a(new_n291), .b(new_n293), .out0(new_n294));
  aoi022aa1n03x5               g199(.a(new_n292), .b(new_n293), .c(new_n289), .d(new_n294), .o1(\s[29] ));
  xorb03aa1n02x5               g200(.a(new_n103), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n02x4               g201(.a(new_n285), .b(new_n281), .c(new_n293), .out0(new_n297));
  aoai13aa1n03x5               g202(.a(new_n297), .b(new_n279), .c(new_n205), .d(new_n271), .o1(new_n298));
  inv000aa1d42x5               g203(.a(new_n297), .o1(new_n299));
  oao003aa1n02x5               g204(.a(\a[29] ), .b(\b[28] ), .c(new_n291), .carry(new_n300));
  aoai13aa1n02x5               g205(.a(new_n300), .b(new_n299), .c(new_n275), .d(new_n272), .o1(new_n301));
  xorc02aa1n02x5               g206(.a(\a[30] ), .b(\b[29] ), .out0(new_n302));
  norb02aa1n02x5               g207(.a(new_n300), .b(new_n302), .out0(new_n303));
  aoi022aa1n02x7               g208(.a(new_n301), .b(new_n302), .c(new_n298), .d(new_n303), .o1(\s[30] ));
  nano32aa1n02x5               g209(.a(new_n285), .b(new_n302), .c(new_n281), .d(new_n293), .out0(new_n305));
  aoai13aa1n02x5               g210(.a(new_n305), .b(new_n279), .c(new_n205), .d(new_n271), .o1(new_n306));
  xorc02aa1n02x5               g211(.a(\a[31] ), .b(\b[30] ), .out0(new_n307));
  and002aa1n02x5               g212(.a(\b[29] ), .b(\a[30] ), .o(new_n308));
  oabi12aa1n02x5               g213(.a(new_n307), .b(\a[30] ), .c(\b[29] ), .out0(new_n309));
  oab012aa1n02x4               g214(.a(new_n309), .b(new_n300), .c(new_n308), .out0(new_n310));
  inv000aa1d42x5               g215(.a(new_n305), .o1(new_n311));
  oao003aa1n02x5               g216(.a(\a[30] ), .b(\b[29] ), .c(new_n300), .carry(new_n312));
  aoai13aa1n02x5               g217(.a(new_n312), .b(new_n311), .c(new_n275), .d(new_n272), .o1(new_n313));
  aoi022aa1n03x5               g218(.a(new_n313), .b(new_n307), .c(new_n306), .d(new_n310), .o1(\s[31] ));
  inv000aa1d42x5               g219(.a(\a[3] ), .o1(new_n315));
  xorb03aa1n02x5               g220(.a(new_n144), .b(\b[2] ), .c(new_n315), .out0(\s[3] ));
  nanp02aa1n02x5               g221(.a(new_n104), .b(new_n106), .o1(new_n317));
  aoib12aa1n02x5               g222(.a(new_n105), .b(new_n315), .c(\b[2] ), .out0(new_n318));
  aoi022aa1n02x5               g223(.a(new_n147), .b(new_n108), .c(new_n318), .d(new_n317), .o1(\s[4] ));
  xnbna2aa1n03x5               g224(.a(new_n112), .b(new_n107), .c(new_n110), .out0(\s[5] ));
  norp02aa1n02x5               g225(.a(\b[4] ), .b(\a[5] ), .o1(new_n321));
  aoi012aa1n02x5               g226(.a(new_n321), .b(new_n147), .c(new_n112), .o1(new_n322));
  xnrb03aa1n02x5               g227(.a(new_n322), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  nanb02aa1n02x5               g228(.a(new_n111), .b(new_n322), .out0(new_n324));
  xnbna2aa1n03x5               g229(.a(new_n116), .b(new_n324), .c(new_n118), .out0(\s[7] ));
  nanp02aa1n02x5               g230(.a(new_n324), .b(new_n119), .o1(new_n326));
  xnbna2aa1n03x5               g231(.a(new_n113), .b(new_n326), .c(new_n121), .out0(\s[8] ));
  xorb03aa1n02x5               g232(.a(new_n124), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


