// Benchmark "adder" written by ABC on Thu Jul 18 15:12:12 2024

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
    new_n118, new_n119, new_n120, new_n121, new_n123, new_n124, new_n125,
    new_n126, new_n127, new_n128, new_n129, new_n130, new_n131, new_n132,
    new_n133, new_n134, new_n136, new_n138, new_n139, new_n140, new_n141,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n148, new_n149,
    new_n150, new_n152, new_n153, new_n154, new_n155, new_n156, new_n157,
    new_n158, new_n159, new_n160, new_n161, new_n162, new_n164, new_n165,
    new_n166, new_n167, new_n168, new_n170, new_n171, new_n172, new_n173,
    new_n174, new_n175, new_n176, new_n178, new_n179, new_n180, new_n181,
    new_n183, new_n184, new_n185, new_n186, new_n187, new_n188, new_n189,
    new_n190, new_n191, new_n192, new_n193, new_n194, new_n195, new_n196,
    new_n197, new_n198, new_n199, new_n200, new_n201, new_n202, new_n205,
    new_n206, new_n207, new_n208, new_n209, new_n210, new_n211, new_n212,
    new_n213, new_n214, new_n216, new_n217, new_n218, new_n219, new_n220,
    new_n221, new_n222, new_n223, new_n224, new_n225, new_n226, new_n228,
    new_n229, new_n230, new_n231, new_n232, new_n233, new_n235, new_n236,
    new_n237, new_n238, new_n239, new_n240, new_n241, new_n242, new_n244,
    new_n245, new_n246, new_n247, new_n248, new_n249, new_n250, new_n251,
    new_n253, new_n254, new_n255, new_n256, new_n257, new_n258, new_n259,
    new_n260, new_n261, new_n262, new_n263, new_n264, new_n266, new_n267,
    new_n268, new_n269, new_n270, new_n271, new_n273, new_n274, new_n275,
    new_n276, new_n277, new_n278, new_n280, new_n281, new_n282, new_n283,
    new_n284, new_n285, new_n286, new_n287, new_n288, new_n289, new_n291,
    new_n292, new_n293, new_n294, new_n295, new_n296, new_n297, new_n300,
    new_n301, new_n302, new_n303, new_n304, new_n305, new_n306, new_n308,
    new_n309, new_n310, new_n311, new_n312, new_n313, new_n314, new_n317,
    new_n320, new_n322, new_n323, new_n324, new_n325, new_n326, new_n328;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nand42aa1n20x5               g001(.a(\b[7] ), .b(\a[8] ), .o1(new_n97));
  norp02aa1n04x5               g002(.a(\b[7] ), .b(\a[8] ), .o1(new_n98));
  nor002aa1d32x5               g003(.a(\b[6] ), .b(\a[7] ), .o1(new_n99));
  oai012aa1n02x5               g004(.a(new_n97), .b(new_n99), .c(new_n98), .o1(new_n100));
  nand42aa1n16x5               g005(.a(\b[6] ), .b(\a[7] ), .o1(new_n101));
  nona23aa1n06x5               g006(.a(new_n97), .b(new_n101), .c(new_n99), .d(new_n98), .out0(new_n102));
  orn002aa1n02x5               g007(.a(\a[5] ), .b(\b[4] ), .o(new_n103));
  oaoi03aa1n02x5               g008(.a(\a[6] ), .b(\b[5] ), .c(new_n103), .o1(new_n104));
  oaib12aa1n06x5               g009(.a(new_n100), .b(new_n102), .c(new_n104), .out0(new_n105));
  nor042aa1n02x5               g010(.a(\b[3] ), .b(\a[4] ), .o1(new_n106));
  nanp02aa1n12x5               g011(.a(\b[3] ), .b(\a[4] ), .o1(new_n107));
  nor042aa1n06x5               g012(.a(\b[2] ), .b(\a[3] ), .o1(new_n108));
  nand42aa1n03x5               g013(.a(\b[2] ), .b(\a[3] ), .o1(new_n109));
  nona23aa1n03x5               g014(.a(new_n109), .b(new_n107), .c(new_n106), .d(new_n108), .out0(new_n110));
  inv000aa1d42x5               g015(.a(\a[2] ), .o1(new_n111));
  inv000aa1d42x5               g016(.a(\b[1] ), .o1(new_n112));
  nanp02aa1n03x5               g017(.a(\b[0] ), .b(\a[1] ), .o1(new_n113));
  tech160nm_fioaoi03aa1n02p5x5 g018(.a(new_n111), .b(new_n112), .c(new_n113), .o1(new_n114));
  aoi012aa1n02x5               g019(.a(new_n106), .b(new_n108), .c(new_n107), .o1(new_n115));
  oai012aa1n06x5               g020(.a(new_n115), .b(new_n110), .c(new_n114), .o1(new_n116));
  tech160nm_fixorc02aa1n04x5   g021(.a(\a[6] ), .b(\b[5] ), .out0(new_n117));
  xorc02aa1n02x5               g022(.a(\a[5] ), .b(\b[4] ), .out0(new_n118));
  nano22aa1n03x7               g023(.a(new_n102), .b(new_n117), .c(new_n118), .out0(new_n119));
  tech160nm_fiaoi012aa1n05x5   g024(.a(new_n105), .b(new_n119), .c(new_n116), .o1(new_n120));
  oaoi03aa1n02x5               g025(.a(\a[9] ), .b(\b[8] ), .c(new_n120), .o1(new_n121));
  xorb03aa1n02x5               g026(.a(new_n121), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  nor022aa1n16x5               g027(.a(\b[10] ), .b(\a[11] ), .o1(new_n123));
  nand42aa1n08x5               g028(.a(\b[10] ), .b(\a[11] ), .o1(new_n124));
  norb02aa1n02x5               g029(.a(new_n124), .b(new_n123), .out0(new_n125));
  inv000aa1n02x5               g030(.a(new_n125), .o1(new_n126));
  nor002aa1d24x5               g031(.a(\b[8] ), .b(\a[9] ), .o1(new_n127));
  nor002aa1d32x5               g032(.a(\b[9] ), .b(\a[10] ), .o1(new_n128));
  nanp02aa1n12x5               g033(.a(\b[9] ), .b(\a[10] ), .o1(new_n129));
  oai012aa1d24x5               g034(.a(new_n129), .b(new_n128), .c(new_n127), .o1(new_n130));
  nanp02aa1n04x5               g035(.a(\b[8] ), .b(\a[9] ), .o1(new_n131));
  nona23aa1d24x5               g036(.a(new_n129), .b(new_n131), .c(new_n127), .d(new_n128), .out0(new_n132));
  oaoi13aa1n02x5               g037(.a(new_n126), .b(new_n130), .c(new_n120), .d(new_n132), .o1(new_n133));
  oai112aa1n02x5               g038(.a(new_n130), .b(new_n126), .c(new_n120), .d(new_n132), .o1(new_n134));
  norb02aa1n02x5               g039(.a(new_n134), .b(new_n133), .out0(\s[11] ));
  norp02aa1n02x5               g040(.a(new_n133), .b(new_n123), .o1(new_n136));
  xnrb03aa1n02x5               g041(.a(new_n136), .b(\b[11] ), .c(\a[12] ), .out0(\s[12] ));
  nor002aa1d32x5               g042(.a(\b[11] ), .b(\a[12] ), .o1(new_n138));
  nand22aa1n12x5               g043(.a(\b[11] ), .b(\a[12] ), .o1(new_n139));
  nona23aa1d18x5               g044(.a(new_n139), .b(new_n124), .c(new_n123), .d(new_n138), .out0(new_n140));
  ao0012aa1n12x5               g045(.a(new_n138), .b(new_n123), .c(new_n139), .o(new_n141));
  oabi12aa1n18x5               g046(.a(new_n141), .b(new_n140), .c(new_n130), .out0(new_n142));
  inv000aa1d42x5               g047(.a(new_n142), .o1(new_n143));
  norp02aa1n02x5               g048(.a(new_n140), .b(new_n132), .o1(new_n144));
  aoai13aa1n03x5               g049(.a(new_n144), .b(new_n105), .c(new_n119), .d(new_n116), .o1(new_n145));
  nand22aa1n03x5               g050(.a(new_n145), .b(new_n143), .o1(new_n146));
  xorb03aa1n02x5               g051(.a(new_n146), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  nor042aa1n04x5               g052(.a(\b[12] ), .b(\a[13] ), .o1(new_n148));
  nand42aa1n06x5               g053(.a(\b[12] ), .b(\a[13] ), .o1(new_n149));
  aoi012aa1n02x5               g054(.a(new_n148), .b(new_n146), .c(new_n149), .o1(new_n150));
  xnrb03aa1n02x5               g055(.a(new_n150), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  nor042aa1n04x5               g056(.a(\b[14] ), .b(\a[15] ), .o1(new_n152));
  nand42aa1d28x5               g057(.a(\b[14] ), .b(\a[15] ), .o1(new_n153));
  nanb02aa1n02x5               g058(.a(new_n152), .b(new_n153), .out0(new_n154));
  inv000aa1d42x5               g059(.a(new_n154), .o1(new_n155));
  nor042aa1n04x5               g060(.a(\b[13] ), .b(\a[14] ), .o1(new_n156));
  nand42aa1n08x5               g061(.a(\b[13] ), .b(\a[14] ), .o1(new_n157));
  aoi012aa1n12x5               g062(.a(new_n156), .b(new_n148), .c(new_n157), .o1(new_n158));
  inv000aa1d42x5               g063(.a(new_n158), .o1(new_n159));
  nano23aa1d15x5               g064(.a(new_n148), .b(new_n156), .c(new_n157), .d(new_n149), .out0(new_n160));
  aoai13aa1n06x5               g065(.a(new_n155), .b(new_n159), .c(new_n146), .d(new_n160), .o1(new_n161));
  aoi112aa1n02x5               g066(.a(new_n159), .b(new_n155), .c(new_n146), .d(new_n160), .o1(new_n162));
  norb02aa1n02x5               g067(.a(new_n161), .b(new_n162), .out0(\s[15] ));
  nor042aa1n04x5               g068(.a(\b[15] ), .b(\a[16] ), .o1(new_n164));
  nand42aa1d28x5               g069(.a(\b[15] ), .b(\a[16] ), .o1(new_n165));
  nanb02aa1n02x5               g070(.a(new_n164), .b(new_n165), .out0(new_n166));
  oai112aa1n03x5               g071(.a(new_n161), .b(new_n166), .c(\b[14] ), .d(\a[15] ), .o1(new_n167));
  oaoi13aa1n06x5               g072(.a(new_n166), .b(new_n161), .c(\a[15] ), .d(\b[14] ), .o1(new_n168));
  norb02aa1n03x4               g073(.a(new_n167), .b(new_n168), .out0(\s[16] ));
  nano23aa1n03x7               g074(.a(new_n123), .b(new_n138), .c(new_n139), .d(new_n124), .out0(new_n170));
  nano23aa1d15x5               g075(.a(new_n152), .b(new_n164), .c(new_n165), .d(new_n153), .out0(new_n171));
  nano32aa1n03x7               g076(.a(new_n132), .b(new_n171), .c(new_n170), .d(new_n160), .out0(new_n172));
  aoai13aa1n06x5               g077(.a(new_n172), .b(new_n105), .c(new_n119), .d(new_n116), .o1(new_n173));
  aoai13aa1n12x5               g078(.a(new_n171), .b(new_n159), .c(new_n142), .d(new_n160), .o1(new_n174));
  tech160nm_fiaoi012aa1n05x5   g079(.a(new_n164), .b(new_n152), .c(new_n165), .o1(new_n175));
  nand23aa1n09x5               g080(.a(new_n173), .b(new_n174), .c(new_n175), .o1(new_n176));
  xorb03aa1n02x5               g081(.a(new_n176), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv000aa1d42x5               g082(.a(\a[18] ), .o1(new_n178));
  inv000aa1d42x5               g083(.a(\a[17] ), .o1(new_n179));
  inv000aa1d42x5               g084(.a(\b[16] ), .o1(new_n180));
  oaoi03aa1n03x5               g085(.a(new_n179), .b(new_n180), .c(new_n176), .o1(new_n181));
  xorb03aa1n02x5               g086(.a(new_n181), .b(\b[17] ), .c(new_n178), .out0(\s[18] ));
  nano23aa1n02x5               g087(.a(new_n99), .b(new_n98), .c(new_n101), .d(new_n97), .out0(new_n183));
  aobi12aa1n02x7               g088(.a(new_n100), .b(new_n183), .c(new_n104), .out0(new_n184));
  nano23aa1n02x4               g089(.a(new_n106), .b(new_n108), .c(new_n109), .d(new_n107), .out0(new_n185));
  oao003aa1n02x5               g090(.a(new_n111), .b(new_n112), .c(new_n113), .carry(new_n186));
  aobi12aa1n06x5               g091(.a(new_n115), .b(new_n185), .c(new_n186), .out0(new_n187));
  nand03aa1n02x5               g092(.a(new_n183), .b(new_n117), .c(new_n118), .o1(new_n188));
  nona23aa1d18x5               g093(.a(new_n160), .b(new_n171), .c(new_n140), .d(new_n132), .out0(new_n189));
  oaoi13aa1n12x5               g094(.a(new_n189), .b(new_n184), .c(new_n187), .d(new_n188), .o1(new_n190));
  inv000aa1d42x5               g095(.a(new_n171), .o1(new_n191));
  inv000aa1d42x5               g096(.a(new_n130), .o1(new_n192));
  aoai13aa1n04x5               g097(.a(new_n160), .b(new_n141), .c(new_n170), .d(new_n192), .o1(new_n193));
  aoai13aa1n06x5               g098(.a(new_n175), .b(new_n191), .c(new_n193), .d(new_n158), .o1(new_n194));
  xroi22aa1d06x4               g099(.a(new_n179), .b(\b[16] ), .c(new_n178), .d(\b[17] ), .out0(new_n195));
  tech160nm_fioai012aa1n03p5x5 g100(.a(new_n195), .b(new_n194), .c(new_n190), .o1(new_n196));
  oaih22aa1n04x5               g101(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n197));
  oaib12aa1n06x5               g102(.a(new_n197), .b(new_n178), .c(\b[17] ), .out0(new_n198));
  nor002aa1n16x5               g103(.a(\b[18] ), .b(\a[19] ), .o1(new_n199));
  nanp02aa1n04x5               g104(.a(\b[18] ), .b(\a[19] ), .o1(new_n200));
  nanb02aa1n02x5               g105(.a(new_n199), .b(new_n200), .out0(new_n201));
  inv000aa1d42x5               g106(.a(new_n201), .o1(new_n202));
  xnbna2aa1n03x5               g107(.a(new_n202), .b(new_n196), .c(new_n198), .out0(\s[19] ));
  xnrc02aa1n02x5               g108(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  inv000aa1d42x5               g109(.a(new_n199), .o1(new_n205));
  tech160nm_fiaoi012aa1n02p5x5 g110(.a(new_n201), .b(new_n196), .c(new_n198), .o1(new_n206));
  nor042aa1n06x5               g111(.a(\b[19] ), .b(\a[20] ), .o1(new_n207));
  nanp02aa1n09x5               g112(.a(\b[19] ), .b(\a[20] ), .o1(new_n208));
  nanb02aa1n02x5               g113(.a(new_n207), .b(new_n208), .out0(new_n209));
  nano22aa1n03x5               g114(.a(new_n206), .b(new_n205), .c(new_n209), .out0(new_n210));
  nanp02aa1n02x5               g115(.a(new_n180), .b(new_n179), .o1(new_n211));
  oaoi03aa1n02x5               g116(.a(\a[18] ), .b(\b[17] ), .c(new_n211), .o1(new_n212));
  aoai13aa1n02x7               g117(.a(new_n202), .b(new_n212), .c(new_n176), .d(new_n195), .o1(new_n213));
  tech160nm_fiaoi012aa1n02p5x5 g118(.a(new_n209), .b(new_n213), .c(new_n205), .o1(new_n214));
  norp02aa1n03x5               g119(.a(new_n214), .b(new_n210), .o1(\s[20] ));
  nano23aa1n09x5               g120(.a(new_n199), .b(new_n207), .c(new_n208), .d(new_n200), .out0(new_n216));
  nand02aa1d04x5               g121(.a(new_n195), .b(new_n216), .o1(new_n217));
  inv000aa1d42x5               g122(.a(new_n217), .o1(new_n218));
  oaih12aa1n02x5               g123(.a(new_n218), .b(new_n194), .c(new_n190), .o1(new_n219));
  nona23aa1n09x5               g124(.a(new_n208), .b(new_n200), .c(new_n199), .d(new_n207), .out0(new_n220));
  aoi012aa1n12x5               g125(.a(new_n207), .b(new_n199), .c(new_n208), .o1(new_n221));
  oai012aa1d24x5               g126(.a(new_n221), .b(new_n220), .c(new_n198), .o1(new_n222));
  inv000aa1d42x5               g127(.a(new_n222), .o1(new_n223));
  nor042aa1n09x5               g128(.a(\b[20] ), .b(\a[21] ), .o1(new_n224));
  nanp02aa1n02x5               g129(.a(\b[20] ), .b(\a[21] ), .o1(new_n225));
  norb02aa1n02x5               g130(.a(new_n225), .b(new_n224), .out0(new_n226));
  xnbna2aa1n03x5               g131(.a(new_n226), .b(new_n219), .c(new_n223), .out0(\s[21] ));
  inv000aa1d42x5               g132(.a(new_n224), .o1(new_n228));
  aobi12aa1n02x7               g133(.a(new_n226), .b(new_n219), .c(new_n223), .out0(new_n229));
  tech160nm_fixnrc02aa1n05x5   g134(.a(\b[21] ), .b(\a[22] ), .out0(new_n230));
  nano22aa1n03x5               g135(.a(new_n229), .b(new_n228), .c(new_n230), .out0(new_n231));
  aoai13aa1n03x5               g136(.a(new_n226), .b(new_n222), .c(new_n176), .d(new_n218), .o1(new_n232));
  tech160nm_fiaoi012aa1n02p5x5 g137(.a(new_n230), .b(new_n232), .c(new_n228), .o1(new_n233));
  norp02aa1n03x5               g138(.a(new_n233), .b(new_n231), .o1(\s[22] ));
  nano22aa1n03x7               g139(.a(new_n230), .b(new_n228), .c(new_n225), .out0(new_n235));
  and003aa1n02x5               g140(.a(new_n195), .b(new_n235), .c(new_n216), .o(new_n236));
  tech160nm_fioai012aa1n03p5x5 g141(.a(new_n236), .b(new_n194), .c(new_n190), .o1(new_n237));
  oao003aa1n06x5               g142(.a(\a[22] ), .b(\b[21] ), .c(new_n228), .carry(new_n238));
  inv000aa1d42x5               g143(.a(new_n238), .o1(new_n239));
  aoi012aa1n02x5               g144(.a(new_n239), .b(new_n222), .c(new_n235), .o1(new_n240));
  xnrc02aa1n12x5               g145(.a(\b[22] ), .b(\a[23] ), .out0(new_n241));
  inv000aa1d42x5               g146(.a(new_n241), .o1(new_n242));
  xnbna2aa1n03x5               g147(.a(new_n242), .b(new_n237), .c(new_n240), .out0(\s[23] ));
  nor042aa1n03x5               g148(.a(\b[22] ), .b(\a[23] ), .o1(new_n244));
  inv000aa1d42x5               g149(.a(new_n244), .o1(new_n245));
  tech160nm_fiaoi012aa1n03p5x5 g150(.a(new_n241), .b(new_n237), .c(new_n240), .o1(new_n246));
  tech160nm_fixnrc02aa1n04x5   g151(.a(\b[23] ), .b(\a[24] ), .out0(new_n247));
  nano22aa1n03x7               g152(.a(new_n246), .b(new_n245), .c(new_n247), .out0(new_n248));
  inv030aa1n02x5               g153(.a(new_n240), .o1(new_n249));
  aoai13aa1n06x5               g154(.a(new_n242), .b(new_n249), .c(new_n176), .d(new_n236), .o1(new_n250));
  tech160nm_fiaoi012aa1n05x5   g155(.a(new_n247), .b(new_n250), .c(new_n245), .o1(new_n251));
  norp02aa1n03x5               g156(.a(new_n251), .b(new_n248), .o1(\s[24] ));
  nor042aa1n02x5               g157(.a(new_n247), .b(new_n241), .o1(new_n253));
  nano22aa1n06x5               g158(.a(new_n217), .b(new_n235), .c(new_n253), .out0(new_n254));
  oaih12aa1n02x5               g159(.a(new_n254), .b(new_n194), .c(new_n190), .o1(new_n255));
  inv000aa1n02x5               g160(.a(new_n221), .o1(new_n256));
  aoai13aa1n06x5               g161(.a(new_n235), .b(new_n256), .c(new_n216), .d(new_n212), .o1(new_n257));
  inv030aa1n02x5               g162(.a(new_n253), .o1(new_n258));
  oao003aa1n02x5               g163(.a(\a[24] ), .b(\b[23] ), .c(new_n245), .carry(new_n259));
  aoai13aa1n06x5               g164(.a(new_n259), .b(new_n258), .c(new_n257), .d(new_n238), .o1(new_n260));
  xnrc02aa1n12x5               g165(.a(\b[24] ), .b(\a[25] ), .out0(new_n261));
  aoib12aa1n06x5               g166(.a(new_n261), .b(new_n255), .c(new_n260), .out0(new_n262));
  inv000aa1d42x5               g167(.a(new_n261), .o1(new_n263));
  aoi112aa1n02x5               g168(.a(new_n263), .b(new_n260), .c(new_n176), .d(new_n254), .o1(new_n264));
  norp02aa1n02x5               g169(.a(new_n262), .b(new_n264), .o1(\s[25] ));
  nor042aa1n03x5               g170(.a(\b[24] ), .b(\a[25] ), .o1(new_n266));
  inv000aa1d42x5               g171(.a(new_n266), .o1(new_n267));
  xnrc02aa1n02x5               g172(.a(\b[25] ), .b(\a[26] ), .out0(new_n268));
  nano22aa1n03x7               g173(.a(new_n262), .b(new_n267), .c(new_n268), .out0(new_n269));
  aoai13aa1n03x5               g174(.a(new_n263), .b(new_n260), .c(new_n176), .d(new_n254), .o1(new_n270));
  tech160nm_fiaoi012aa1n02p5x5 g175(.a(new_n268), .b(new_n270), .c(new_n267), .o1(new_n271));
  nor002aa1n02x5               g176(.a(new_n271), .b(new_n269), .o1(\s[26] ));
  nor042aa1n04x5               g177(.a(new_n268), .b(new_n261), .o1(new_n273));
  nano32aa1n03x7               g178(.a(new_n217), .b(new_n273), .c(new_n235), .d(new_n253), .out0(new_n274));
  oai012aa1n06x5               g179(.a(new_n274), .b(new_n194), .c(new_n190), .o1(new_n275));
  oao003aa1n02x5               g180(.a(\a[26] ), .b(\b[25] ), .c(new_n267), .carry(new_n276));
  aobi12aa1n06x5               g181(.a(new_n276), .b(new_n260), .c(new_n273), .out0(new_n277));
  xorc02aa1n12x5               g182(.a(\a[27] ), .b(\b[26] ), .out0(new_n278));
  xnbna2aa1n03x5               g183(.a(new_n278), .b(new_n275), .c(new_n277), .out0(\s[27] ));
  norp02aa1n02x5               g184(.a(\b[26] ), .b(\a[27] ), .o1(new_n280));
  inv040aa1n03x5               g185(.a(new_n280), .o1(new_n281));
  aobi12aa1n02x7               g186(.a(new_n278), .b(new_n275), .c(new_n277), .out0(new_n282));
  xnrc02aa1n02x5               g187(.a(\b[27] ), .b(\a[28] ), .out0(new_n283));
  nano22aa1n03x5               g188(.a(new_n282), .b(new_n281), .c(new_n283), .out0(new_n284));
  aoai13aa1n06x5               g189(.a(new_n253), .b(new_n239), .c(new_n222), .d(new_n235), .o1(new_n285));
  inv000aa1d42x5               g190(.a(new_n273), .o1(new_n286));
  aoai13aa1n04x5               g191(.a(new_n276), .b(new_n286), .c(new_n285), .d(new_n259), .o1(new_n287));
  aoai13aa1n03x5               g192(.a(new_n278), .b(new_n287), .c(new_n176), .d(new_n274), .o1(new_n288));
  tech160nm_fiaoi012aa1n02p5x5 g193(.a(new_n283), .b(new_n288), .c(new_n281), .o1(new_n289));
  norp02aa1n03x5               g194(.a(new_n289), .b(new_n284), .o1(\s[28] ));
  norb02aa1n02x5               g195(.a(new_n278), .b(new_n283), .out0(new_n291));
  aobi12aa1n02x7               g196(.a(new_n291), .b(new_n275), .c(new_n277), .out0(new_n292));
  oao003aa1n02x5               g197(.a(\a[28] ), .b(\b[27] ), .c(new_n281), .carry(new_n293));
  xnrc02aa1n02x5               g198(.a(\b[28] ), .b(\a[29] ), .out0(new_n294));
  nano22aa1n03x5               g199(.a(new_n292), .b(new_n293), .c(new_n294), .out0(new_n295));
  aoai13aa1n03x5               g200(.a(new_n291), .b(new_n287), .c(new_n176), .d(new_n274), .o1(new_n296));
  tech160nm_fiaoi012aa1n02p5x5 g201(.a(new_n294), .b(new_n296), .c(new_n293), .o1(new_n297));
  norp02aa1n03x5               g202(.a(new_n297), .b(new_n295), .o1(\s[29] ));
  xorb03aa1n02x5               g203(.a(new_n113), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g204(.a(new_n278), .b(new_n294), .c(new_n283), .out0(new_n300));
  aobi12aa1n02x7               g205(.a(new_n300), .b(new_n275), .c(new_n277), .out0(new_n301));
  oao003aa1n02x5               g206(.a(\a[29] ), .b(\b[28] ), .c(new_n293), .carry(new_n302));
  xnrc02aa1n02x5               g207(.a(\b[29] ), .b(\a[30] ), .out0(new_n303));
  nano22aa1n03x5               g208(.a(new_n301), .b(new_n302), .c(new_n303), .out0(new_n304));
  aoai13aa1n03x5               g209(.a(new_n300), .b(new_n287), .c(new_n176), .d(new_n274), .o1(new_n305));
  tech160nm_fiaoi012aa1n02p5x5 g210(.a(new_n303), .b(new_n305), .c(new_n302), .o1(new_n306));
  norp02aa1n03x5               g211(.a(new_n306), .b(new_n304), .o1(\s[30] ));
  xnrc02aa1n02x5               g212(.a(\b[30] ), .b(\a[31] ), .out0(new_n308));
  norb02aa1n02x5               g213(.a(new_n300), .b(new_n303), .out0(new_n309));
  aobi12aa1n02x7               g214(.a(new_n309), .b(new_n275), .c(new_n277), .out0(new_n310));
  oao003aa1n02x5               g215(.a(\a[30] ), .b(\b[29] ), .c(new_n302), .carry(new_n311));
  nano22aa1n03x5               g216(.a(new_n310), .b(new_n308), .c(new_n311), .out0(new_n312));
  aoai13aa1n03x5               g217(.a(new_n309), .b(new_n287), .c(new_n176), .d(new_n274), .o1(new_n313));
  tech160nm_fiaoi012aa1n02p5x5 g218(.a(new_n308), .b(new_n313), .c(new_n311), .o1(new_n314));
  norp02aa1n03x5               g219(.a(new_n314), .b(new_n312), .o1(\s[31] ));
  xnrb03aa1n02x5               g220(.a(new_n114), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g221(.a(\a[3] ), .b(\b[2] ), .c(new_n114), .o1(new_n317));
  xorb03aa1n02x5               g222(.a(new_n317), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g223(.a(new_n116), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oao003aa1n03x5               g224(.a(\a[5] ), .b(\b[4] ), .c(new_n187), .carry(new_n320));
  xnrc02aa1n02x5               g225(.a(new_n320), .b(new_n117), .out0(\s[6] ));
  and002aa1n02x5               g226(.a(\b[5] ), .b(\a[6] ), .o(new_n322));
  nanp02aa1n03x5               g227(.a(new_n320), .b(new_n117), .o1(new_n323));
  nona23aa1n06x5               g228(.a(new_n323), .b(new_n101), .c(new_n99), .d(new_n322), .out0(new_n324));
  inv000aa1d42x5               g229(.a(new_n99), .o1(new_n325));
  aboi22aa1n03x5               g230(.a(new_n322), .b(new_n323), .c(new_n325), .d(new_n101), .out0(new_n326));
  norb02aa1n02x5               g231(.a(new_n324), .b(new_n326), .out0(\s[7] ));
  norb02aa1n02x5               g232(.a(new_n97), .b(new_n98), .out0(new_n328));
  xnbna2aa1n03x5               g233(.a(new_n328), .b(new_n324), .c(new_n325), .out0(\s[8] ));
  xnrb03aa1n02x5               g234(.a(new_n120), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


