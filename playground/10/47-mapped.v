// Benchmark "adder" written by ABC on Wed Jul 17 17:30:45 2024

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
    new_n133, new_n135, new_n136, new_n137, new_n138, new_n139, new_n140,
    new_n141, new_n142, new_n143, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n152, new_n153, new_n154, new_n155, new_n156,
    new_n157, new_n158, new_n159, new_n161, new_n162, new_n163, new_n164,
    new_n165, new_n166, new_n167, new_n169, new_n170, new_n171, new_n172,
    new_n173, new_n174, new_n175, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n190, new_n191, new_n192, new_n193, new_n195, new_n196,
    new_n197, new_n198, new_n199, new_n200, new_n201, new_n202, new_n203,
    new_n206, new_n207, new_n208, new_n209, new_n210, new_n211, new_n212,
    new_n213, new_n215, new_n216, new_n217, new_n218, new_n219, new_n220,
    new_n221, new_n222, new_n223, new_n225, new_n226, new_n227, new_n228,
    new_n229, new_n230, new_n231, new_n233, new_n234, new_n235, new_n236,
    new_n237, new_n238, new_n239, new_n240, new_n241, new_n242, new_n243,
    new_n244, new_n245, new_n247, new_n248, new_n249, new_n250, new_n252,
    new_n253, new_n254, new_n255, new_n256, new_n257, new_n258, new_n259,
    new_n260, new_n262, new_n263, new_n264, new_n265, new_n266, new_n267,
    new_n268, new_n270, new_n271, new_n272, new_n273, new_n274, new_n275,
    new_n276, new_n277, new_n279, new_n280, new_n281, new_n282, new_n283,
    new_n284, new_n285, new_n286, new_n287, new_n289, new_n290, new_n291,
    new_n292, new_n293, new_n294, new_n295, new_n298, new_n299, new_n300,
    new_n301, new_n302, new_n303, new_n304, new_n306, new_n307, new_n308,
    new_n309, new_n310, new_n311, new_n312, new_n313, new_n314, new_n317,
    new_n320, new_n322, new_n323, new_n325, new_n326, new_n327, new_n328;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  norp02aa1n24x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  nor042aa1n02x5               g002(.a(\b[3] ), .b(\a[4] ), .o1(new_n98));
  nand42aa1d28x5               g003(.a(\b[3] ), .b(\a[4] ), .o1(new_n99));
  nor022aa1n16x5               g004(.a(\b[2] ), .b(\a[3] ), .o1(new_n100));
  tech160nm_fiao0012aa1n02p5x5 g005(.a(new_n98), .b(new_n100), .c(new_n99), .o(new_n101));
  nanp02aa1n02x5               g006(.a(\b[1] ), .b(\a[2] ), .o1(new_n102));
  nand22aa1n09x5               g007(.a(\b[0] ), .b(\a[1] ), .o1(new_n103));
  norp02aa1n02x5               g008(.a(\b[1] ), .b(\a[2] ), .o1(new_n104));
  oa0012aa1n06x5               g009(.a(new_n102), .b(new_n104), .c(new_n103), .o(new_n105));
  nand42aa1n08x5               g010(.a(\b[2] ), .b(\a[3] ), .o1(new_n106));
  nano23aa1n03x7               g011(.a(new_n98), .b(new_n100), .c(new_n106), .d(new_n99), .out0(new_n107));
  aoi012aa1n09x5               g012(.a(new_n101), .b(new_n107), .c(new_n105), .o1(new_n108));
  nand42aa1d28x5               g013(.a(\b[6] ), .b(\a[7] ), .o1(new_n109));
  nor022aa1n04x5               g014(.a(\b[6] ), .b(\a[7] ), .o1(new_n110));
  norp02aa1n02x5               g015(.a(\b[5] ), .b(\a[6] ), .o1(new_n111));
  nand42aa1d28x5               g016(.a(\b[5] ), .b(\a[6] ), .o1(new_n112));
  nano23aa1n03x5               g017(.a(new_n111), .b(new_n110), .c(new_n112), .d(new_n109), .out0(new_n113));
  tech160nm_fixorc02aa1n02p5x5 g018(.a(\a[5] ), .b(\b[4] ), .out0(new_n114));
  tech160nm_fixorc02aa1n02p5x5 g019(.a(\a[8] ), .b(\b[7] ), .out0(new_n115));
  nand23aa1n06x5               g020(.a(new_n113), .b(new_n114), .c(new_n115), .o1(new_n116));
  nanp02aa1n02x5               g021(.a(\b[7] ), .b(\a[8] ), .o1(new_n117));
  oai022aa1n02x5               g022(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n118));
  aoi022aa1n02x5               g023(.a(\b[6] ), .b(\a[7] ), .c(\a[6] ), .d(\b[5] ), .o1(new_n119));
  oai022aa1n02x5               g024(.a(\a[7] ), .b(\b[6] ), .c(\b[7] ), .d(\a[8] ), .o1(new_n120));
  aoai13aa1n12x5               g025(.a(new_n117), .b(new_n120), .c(new_n118), .d(new_n119), .o1(new_n121));
  oai012aa1n12x5               g026(.a(new_n121), .b(new_n108), .c(new_n116), .o1(new_n122));
  nanp02aa1n04x5               g027(.a(\b[8] ), .b(\a[9] ), .o1(new_n123));
  norp02aa1n12x5               g028(.a(\b[9] ), .b(\a[10] ), .o1(new_n124));
  nand02aa1n08x5               g029(.a(\b[9] ), .b(\a[10] ), .o1(new_n125));
  norb02aa1n02x5               g030(.a(new_n125), .b(new_n124), .out0(new_n126));
  aoai13aa1n06x5               g031(.a(new_n126), .b(new_n97), .c(new_n122), .d(new_n123), .o1(new_n127));
  aoi112aa1n02x5               g032(.a(new_n126), .b(new_n97), .c(new_n122), .d(new_n123), .o1(new_n128));
  norb02aa1n02x5               g033(.a(new_n127), .b(new_n128), .out0(\s[10] ));
  oai012aa1n02x5               g034(.a(new_n125), .b(new_n124), .c(new_n97), .o1(new_n130));
  nanp02aa1n04x5               g035(.a(\b[10] ), .b(\a[11] ), .o1(new_n131));
  nor022aa1n16x5               g036(.a(\b[10] ), .b(\a[11] ), .o1(new_n132));
  norb02aa1n02x5               g037(.a(new_n131), .b(new_n132), .out0(new_n133));
  xnbna2aa1n03x5               g038(.a(new_n133), .b(new_n127), .c(new_n130), .out0(\s[11] ));
  inv000aa1n02x5               g039(.a(new_n125), .o1(new_n135));
  oab012aa1n09x5               g040(.a(new_n135), .b(new_n97), .c(new_n124), .out0(new_n136));
  norb02aa1n02x5               g041(.a(new_n133), .b(new_n136), .out0(new_n137));
  aoi022aa1n02x5               g042(.a(new_n127), .b(new_n137), .c(\b[10] ), .d(\a[11] ), .o1(new_n138));
  nor022aa1n08x5               g043(.a(\b[11] ), .b(\a[12] ), .o1(new_n139));
  nand22aa1n04x5               g044(.a(\b[11] ), .b(\a[12] ), .o1(new_n140));
  nanb02aa1n02x5               g045(.a(new_n139), .b(new_n140), .out0(new_n141));
  oaib12aa1n02x5               g046(.a(new_n131), .b(new_n139), .c(new_n140), .out0(new_n142));
  aoi012aa1n02x5               g047(.a(new_n142), .b(new_n127), .c(new_n137), .o1(new_n143));
  oabi12aa1n02x5               g048(.a(new_n143), .b(new_n138), .c(new_n141), .out0(\s[12] ));
  nona23aa1n03x5               g049(.a(new_n125), .b(new_n123), .c(new_n97), .d(new_n124), .out0(new_n145));
  nona23aa1n06x5               g050(.a(new_n131), .b(new_n140), .c(new_n139), .d(new_n132), .out0(new_n146));
  nor042aa1n04x5               g051(.a(new_n146), .b(new_n145), .o1(new_n147));
  tech160nm_fiao0012aa1n02p5x5 g052(.a(new_n139), .b(new_n132), .c(new_n140), .o(new_n148));
  oabi12aa1n02x5               g053(.a(new_n148), .b(new_n146), .c(new_n130), .out0(new_n149));
  tech160nm_fiao0012aa1n03p5x5 g054(.a(new_n149), .b(new_n122), .c(new_n147), .o(new_n150));
  xorb03aa1n02x5               g055(.a(new_n150), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  nor022aa1n04x5               g056(.a(\b[13] ), .b(\a[14] ), .o1(new_n152));
  nand02aa1n04x5               g057(.a(\b[13] ), .b(\a[14] ), .o1(new_n153));
  nanb02aa1n02x5               g058(.a(new_n152), .b(new_n153), .out0(new_n154));
  nanp02aa1n04x5               g059(.a(\b[12] ), .b(\a[13] ), .o1(new_n155));
  nor022aa1n08x5               g060(.a(\b[12] ), .b(\a[13] ), .o1(new_n156));
  inv000aa1d42x5               g061(.a(new_n154), .o1(new_n157));
  aoi112aa1n02x5               g062(.a(new_n157), .b(new_n156), .c(new_n150), .d(new_n155), .o1(new_n158));
  aoi012aa1n02x5               g063(.a(new_n156), .b(new_n150), .c(new_n155), .o1(new_n159));
  oab012aa1n02x5               g064(.a(new_n158), .b(new_n159), .c(new_n154), .out0(\s[14] ));
  nona23aa1n09x5               g065(.a(new_n155), .b(new_n153), .c(new_n152), .d(new_n156), .out0(new_n161));
  inv000aa1n02x5               g066(.a(new_n161), .o1(new_n162));
  aoai13aa1n03x5               g067(.a(new_n162), .b(new_n149), .c(new_n122), .d(new_n147), .o1(new_n163));
  tech160nm_fioai012aa1n03p5x5 g068(.a(new_n153), .b(new_n152), .c(new_n156), .o1(new_n164));
  nanp02aa1n04x5               g069(.a(\b[14] ), .b(\a[15] ), .o1(new_n165));
  nor002aa1d32x5               g070(.a(\b[14] ), .b(\a[15] ), .o1(new_n166));
  nanb02aa1n02x5               g071(.a(new_n166), .b(new_n165), .out0(new_n167));
  xobna2aa1n03x5               g072(.a(new_n167), .b(new_n163), .c(new_n164), .out0(\s[15] ));
  tech160nm_fiao0012aa1n02p5x5 g073(.a(new_n167), .b(new_n163), .c(new_n164), .o(new_n169));
  nor002aa1d32x5               g074(.a(\b[15] ), .b(\a[16] ), .o1(new_n170));
  nand02aa1n08x5               g075(.a(\b[15] ), .b(\a[16] ), .o1(new_n171));
  nanb02aa1n02x5               g076(.a(new_n170), .b(new_n171), .out0(new_n172));
  aoib12aa1n02x5               g077(.a(new_n166), .b(new_n171), .c(new_n170), .out0(new_n173));
  inv000aa1d42x5               g078(.a(new_n166), .o1(new_n174));
  aoai13aa1n02x5               g079(.a(new_n174), .b(new_n167), .c(new_n163), .d(new_n164), .o1(new_n175));
  aboi22aa1n03x5               g080(.a(new_n172), .b(new_n175), .c(new_n169), .d(new_n173), .out0(\s[16] ));
  nor042aa1d18x5               g081(.a(\b[16] ), .b(\a[17] ), .o1(new_n177));
  inv040aa1n02x5               g082(.a(new_n177), .o1(new_n178));
  nand42aa1n02x5               g083(.a(\b[16] ), .b(\a[17] ), .o1(new_n179));
  nona23aa1d24x5               g084(.a(new_n165), .b(new_n171), .c(new_n170), .d(new_n166), .out0(new_n180));
  nona22aa1n09x5               g085(.a(new_n147), .b(new_n161), .c(new_n180), .out0(new_n181));
  nanb02aa1n06x5               g086(.a(new_n181), .b(new_n122), .out0(new_n182));
  inv000aa1d42x5               g087(.a(new_n180), .o1(new_n183));
  nano23aa1n03x7               g088(.a(new_n139), .b(new_n132), .c(new_n140), .d(new_n131), .out0(new_n184));
  aoai13aa1n06x5               g089(.a(new_n162), .b(new_n148), .c(new_n184), .d(new_n136), .o1(new_n185));
  aob012aa1n02x5               g090(.a(new_n183), .b(new_n185), .c(new_n164), .out0(new_n186));
  aoi012aa1n12x5               g091(.a(new_n170), .b(new_n166), .c(new_n171), .o1(new_n187));
  nanp03aa1d12x5               g092(.a(new_n182), .b(new_n186), .c(new_n187), .o1(new_n188));
  xobna2aa1n03x5               g093(.a(new_n188), .b(new_n179), .c(new_n178), .out0(\s[17] ));
  norb02aa1n02x5               g094(.a(new_n179), .b(new_n177), .out0(new_n190));
  aoai13aa1n06x5               g095(.a(new_n187), .b(new_n180), .c(new_n185), .d(new_n164), .o1(new_n191));
  nanb03aa1n03x5               g096(.a(new_n191), .b(new_n190), .c(new_n182), .out0(new_n192));
  xnrc02aa1n12x5               g097(.a(\b[17] ), .b(\a[18] ), .out0(new_n193));
  xnbna2aa1n03x5               g098(.a(new_n193), .b(new_n192), .c(new_n179), .out0(\s[18] ));
  oaoi13aa1n12x5               g099(.a(new_n181), .b(new_n121), .c(new_n108), .d(new_n116), .o1(new_n195));
  nano22aa1n03x7               g100(.a(new_n193), .b(new_n178), .c(new_n179), .out0(new_n196));
  oai012aa1n06x5               g101(.a(new_n196), .b(new_n191), .c(new_n195), .o1(new_n197));
  oaih22aa1d12x5               g102(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n198));
  aob012aa1n12x5               g103(.a(new_n198), .b(\b[17] ), .c(\a[18] ), .out0(new_n199));
  nand42aa1d28x5               g104(.a(\b[18] ), .b(\a[19] ), .o1(new_n200));
  nor002aa1d32x5               g105(.a(\b[18] ), .b(\a[19] ), .o1(new_n201));
  nanb02aa1n02x5               g106(.a(new_n201), .b(new_n200), .out0(new_n202));
  inv000aa1d42x5               g107(.a(new_n202), .o1(new_n203));
  xnbna2aa1n03x5               g108(.a(new_n203), .b(new_n197), .c(new_n199), .out0(\s[19] ));
  xnrc02aa1n02x5               g109(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  oaoi03aa1n03x5               g110(.a(\a[18] ), .b(\b[17] ), .c(new_n178), .o1(new_n206));
  aoai13aa1n03x5               g111(.a(new_n203), .b(new_n206), .c(new_n188), .d(new_n196), .o1(new_n207));
  nor002aa1n12x5               g112(.a(\b[19] ), .b(\a[20] ), .o1(new_n208));
  nand42aa1d28x5               g113(.a(\b[19] ), .b(\a[20] ), .o1(new_n209));
  norb02aa1n02x5               g114(.a(new_n209), .b(new_n208), .out0(new_n210));
  aoib12aa1n02x5               g115(.a(new_n201), .b(new_n209), .c(new_n208), .out0(new_n211));
  inv040aa1n02x5               g116(.a(new_n201), .o1(new_n212));
  aoai13aa1n02x5               g117(.a(new_n212), .b(new_n202), .c(new_n197), .d(new_n199), .o1(new_n213));
  aoi022aa1n03x5               g118(.a(new_n213), .b(new_n210), .c(new_n207), .d(new_n211), .o1(\s[20] ));
  nano23aa1n09x5               g119(.a(new_n208), .b(new_n201), .c(new_n209), .d(new_n200), .out0(new_n215));
  nand02aa1n06x5               g120(.a(new_n196), .b(new_n215), .o1(new_n216));
  inv000aa1d42x5               g121(.a(new_n216), .o1(new_n217));
  oaih12aa1n02x5               g122(.a(new_n217), .b(new_n191), .c(new_n195), .o1(new_n218));
  nona23aa1d18x5               g123(.a(new_n200), .b(new_n209), .c(new_n208), .d(new_n201), .out0(new_n219));
  oaoi03aa1n09x5               g124(.a(\a[20] ), .b(\b[19] ), .c(new_n212), .o1(new_n220));
  oabi12aa1n18x5               g125(.a(new_n220), .b(new_n219), .c(new_n199), .out0(new_n221));
  inv000aa1d42x5               g126(.a(new_n221), .o1(new_n222));
  xorc02aa1n02x5               g127(.a(\a[21] ), .b(\b[20] ), .out0(new_n223));
  xnbna2aa1n03x5               g128(.a(new_n223), .b(new_n218), .c(new_n222), .out0(\s[21] ));
  aoai13aa1n03x5               g129(.a(new_n223), .b(new_n221), .c(new_n188), .d(new_n217), .o1(new_n225));
  xorc02aa1n02x5               g130(.a(\a[22] ), .b(\b[21] ), .out0(new_n226));
  nor002aa1d32x5               g131(.a(\b[20] ), .b(\a[21] ), .o1(new_n227));
  norp02aa1n02x5               g132(.a(new_n226), .b(new_n227), .o1(new_n228));
  inv000aa1d42x5               g133(.a(new_n227), .o1(new_n229));
  inv000aa1d42x5               g134(.a(new_n223), .o1(new_n230));
  aoai13aa1n03x5               g135(.a(new_n229), .b(new_n230), .c(new_n218), .d(new_n222), .o1(new_n231));
  aoi022aa1n03x5               g136(.a(new_n231), .b(new_n226), .c(new_n225), .d(new_n228), .o1(\s[22] ));
  nanp02aa1n02x5               g137(.a(\b[20] ), .b(\a[21] ), .o1(new_n233));
  xnrc02aa1n02x5               g138(.a(\b[21] ), .b(\a[22] ), .out0(new_n234));
  nano22aa1n03x7               g139(.a(new_n234), .b(new_n233), .c(new_n229), .out0(new_n235));
  and003aa1n02x5               g140(.a(new_n196), .b(new_n235), .c(new_n215), .o(new_n236));
  oaih12aa1n02x5               g141(.a(new_n236), .b(new_n191), .c(new_n195), .o1(new_n237));
  aoai13aa1n06x5               g142(.a(new_n235), .b(new_n220), .c(new_n215), .d(new_n206), .o1(new_n238));
  oao003aa1n12x5               g143(.a(\a[22] ), .b(\b[21] ), .c(new_n229), .carry(new_n239));
  nanp03aa1n02x5               g144(.a(new_n237), .b(new_n238), .c(new_n239), .o1(new_n240));
  nor002aa1n02x5               g145(.a(\b[22] ), .b(\a[23] ), .o1(new_n241));
  and002aa1n12x5               g146(.a(\b[22] ), .b(\a[23] ), .o(new_n242));
  nor042aa1n03x5               g147(.a(new_n242), .b(new_n241), .o1(new_n243));
  inv000aa1d42x5               g148(.a(new_n239), .o1(new_n244));
  aoi112aa1n02x5               g149(.a(new_n243), .b(new_n244), .c(new_n221), .d(new_n235), .o1(new_n245));
  aoi022aa1n02x5               g150(.a(new_n240), .b(new_n243), .c(new_n237), .d(new_n245), .o1(\s[23] ));
  aoi112aa1n02x5               g151(.a(new_n241), .b(new_n244), .c(new_n221), .d(new_n235), .o1(new_n247));
  xorc02aa1n12x5               g152(.a(\a[24] ), .b(\b[23] ), .out0(new_n248));
  aoai13aa1n03x5               g153(.a(new_n248), .b(new_n242), .c(new_n237), .d(new_n247), .o1(new_n249));
  orn002aa1n02x5               g154(.a(new_n248), .b(new_n242), .o(new_n250));
  aoai13aa1n03x5               g155(.a(new_n249), .b(new_n250), .c(new_n247), .d(new_n237), .o1(\s[24] ));
  nano32aa1n02x4               g156(.a(new_n216), .b(new_n248), .c(new_n235), .d(new_n243), .out0(new_n252));
  oaih12aa1n02x5               g157(.a(new_n252), .b(new_n191), .c(new_n195), .o1(new_n253));
  and002aa1n02x5               g158(.a(new_n248), .b(new_n243), .o(new_n254));
  inv000aa1n04x5               g159(.a(new_n254), .o1(new_n255));
  orn002aa1n02x5               g160(.a(\a[23] ), .b(\b[22] ), .o(new_n256));
  oao003aa1n02x5               g161(.a(\a[24] ), .b(\b[23] ), .c(new_n256), .carry(new_n257));
  aoai13aa1n12x5               g162(.a(new_n257), .b(new_n255), .c(new_n238), .d(new_n239), .o1(new_n258));
  inv000aa1d42x5               g163(.a(new_n258), .o1(new_n259));
  xorc02aa1n12x5               g164(.a(\a[25] ), .b(\b[24] ), .out0(new_n260));
  xnbna2aa1n03x5               g165(.a(new_n260), .b(new_n253), .c(new_n259), .out0(\s[25] ));
  aoai13aa1n03x5               g166(.a(new_n260), .b(new_n258), .c(new_n188), .d(new_n252), .o1(new_n262));
  tech160nm_fixorc02aa1n02p5x5 g167(.a(\a[26] ), .b(\b[25] ), .out0(new_n263));
  nor042aa1n03x5               g168(.a(\b[24] ), .b(\a[25] ), .o1(new_n264));
  norp02aa1n02x5               g169(.a(new_n263), .b(new_n264), .o1(new_n265));
  inv000aa1d42x5               g170(.a(new_n264), .o1(new_n266));
  inv000aa1d42x5               g171(.a(new_n260), .o1(new_n267));
  aoai13aa1n02x7               g172(.a(new_n266), .b(new_n267), .c(new_n253), .d(new_n259), .o1(new_n268));
  aoi022aa1n03x5               g173(.a(new_n268), .b(new_n263), .c(new_n262), .d(new_n265), .o1(\s[26] ));
  norp02aa1n02x5               g174(.a(\b[26] ), .b(\a[27] ), .o1(new_n270));
  and002aa1n24x5               g175(.a(\b[26] ), .b(\a[27] ), .o(new_n271));
  norp02aa1n06x5               g176(.a(new_n271), .b(new_n270), .o1(new_n272));
  and002aa1n12x5               g177(.a(new_n263), .b(new_n260), .o(new_n273));
  nano23aa1n06x5               g178(.a(new_n255), .b(new_n216), .c(new_n273), .d(new_n235), .out0(new_n274));
  oai012aa1n06x5               g179(.a(new_n274), .b(new_n191), .c(new_n195), .o1(new_n275));
  oao003aa1n02x5               g180(.a(\a[26] ), .b(\b[25] ), .c(new_n266), .carry(new_n276));
  aobi12aa1n09x5               g181(.a(new_n276), .b(new_n258), .c(new_n273), .out0(new_n277));
  xnbna2aa1n03x5               g182(.a(new_n272), .b(new_n277), .c(new_n275), .out0(\s[27] ));
  inv000aa1d42x5               g183(.a(new_n271), .o1(new_n279));
  aoai13aa1n03x5               g184(.a(new_n254), .b(new_n244), .c(new_n221), .d(new_n235), .o1(new_n280));
  inv000aa1d42x5               g185(.a(new_n273), .o1(new_n281));
  aoai13aa1n02x7               g186(.a(new_n276), .b(new_n281), .c(new_n280), .d(new_n257), .o1(new_n282));
  aoai13aa1n03x5               g187(.a(new_n279), .b(new_n282), .c(new_n188), .d(new_n274), .o1(new_n283));
  xorc02aa1n02x5               g188(.a(\a[28] ), .b(\b[27] ), .out0(new_n284));
  norp02aa1n02x5               g189(.a(new_n284), .b(new_n270), .o1(new_n285));
  inv000aa1n03x5               g190(.a(new_n270), .o1(new_n286));
  aoai13aa1n02x7               g191(.a(new_n286), .b(new_n271), .c(new_n277), .d(new_n275), .o1(new_n287));
  aoi022aa1n03x5               g192(.a(new_n287), .b(new_n284), .c(new_n283), .d(new_n285), .o1(\s[28] ));
  and002aa1n06x5               g193(.a(new_n284), .b(new_n272), .o(new_n289));
  aoai13aa1n03x5               g194(.a(new_n289), .b(new_n282), .c(new_n188), .d(new_n274), .o1(new_n290));
  inv000aa1n06x5               g195(.a(new_n289), .o1(new_n291));
  oao003aa1n02x5               g196(.a(\a[28] ), .b(\b[27] ), .c(new_n286), .carry(new_n292));
  aoai13aa1n02x7               g197(.a(new_n292), .b(new_n291), .c(new_n277), .d(new_n275), .o1(new_n293));
  xorc02aa1n02x5               g198(.a(\a[29] ), .b(\b[28] ), .out0(new_n294));
  norb02aa1n02x5               g199(.a(new_n292), .b(new_n294), .out0(new_n295));
  aoi022aa1n03x5               g200(.a(new_n293), .b(new_n294), .c(new_n290), .d(new_n295), .o1(\s[29] ));
  xorb03aa1n02x5               g201(.a(new_n103), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  and003aa1n02x5               g202(.a(new_n284), .b(new_n294), .c(new_n272), .o(new_n298));
  aoai13aa1n03x5               g203(.a(new_n298), .b(new_n282), .c(new_n188), .d(new_n274), .o1(new_n299));
  inv000aa1d42x5               g204(.a(new_n298), .o1(new_n300));
  oao003aa1n02x5               g205(.a(\a[29] ), .b(\b[28] ), .c(new_n292), .carry(new_n301));
  aoai13aa1n02x7               g206(.a(new_n301), .b(new_n300), .c(new_n277), .d(new_n275), .o1(new_n302));
  xorc02aa1n02x5               g207(.a(\a[30] ), .b(\b[29] ), .out0(new_n303));
  norb02aa1n02x5               g208(.a(new_n301), .b(new_n303), .out0(new_n304));
  aoi022aa1n03x5               g209(.a(new_n302), .b(new_n303), .c(new_n299), .d(new_n304), .o1(\s[30] ));
  xorc02aa1n02x5               g210(.a(\a[31] ), .b(\b[30] ), .out0(new_n306));
  nano22aa1n06x5               g211(.a(new_n291), .b(new_n294), .c(new_n303), .out0(new_n307));
  aoai13aa1n03x5               g212(.a(new_n307), .b(new_n282), .c(new_n188), .d(new_n274), .o1(new_n308));
  inv000aa1d42x5               g213(.a(new_n307), .o1(new_n309));
  oao003aa1n02x5               g214(.a(\a[30] ), .b(\b[29] ), .c(new_n301), .carry(new_n310));
  aoai13aa1n02x7               g215(.a(new_n310), .b(new_n309), .c(new_n277), .d(new_n275), .o1(new_n311));
  and002aa1n02x5               g216(.a(\b[29] ), .b(\a[30] ), .o(new_n312));
  oabi12aa1n02x5               g217(.a(new_n306), .b(\a[30] ), .c(\b[29] ), .out0(new_n313));
  oab012aa1n02x4               g218(.a(new_n313), .b(new_n301), .c(new_n312), .out0(new_n314));
  aoi022aa1n03x5               g219(.a(new_n311), .b(new_n306), .c(new_n308), .d(new_n314), .o1(\s[31] ));
  xorb03aa1n02x5               g220(.a(new_n105), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oai012aa1n02x5               g221(.a(new_n106), .b(new_n105), .c(new_n100), .o1(new_n317));
  xnrb03aa1n02x5               g222(.a(new_n317), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xnrc02aa1n02x5               g223(.a(new_n108), .b(new_n114), .out0(\s[5] ));
  oao003aa1n03x5               g224(.a(\a[5] ), .b(\b[4] ), .c(new_n108), .carry(new_n320));
  xnrb03aa1n02x5               g225(.a(new_n320), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  norb02aa1n02x5               g226(.a(new_n109), .b(new_n110), .out0(new_n322));
  nanb03aa1n06x5               g227(.a(new_n111), .b(new_n320), .c(new_n112), .out0(new_n323));
  xobna2aa1n03x5               g228(.a(new_n322), .b(new_n323), .c(new_n112), .out0(\s[7] ));
  aobi12aa1n02x5               g229(.a(new_n322), .b(new_n323), .c(new_n112), .out0(new_n325));
  aoai13aa1n02x5               g230(.a(new_n109), .b(new_n110), .c(new_n323), .d(new_n112), .o1(new_n326));
  nanp02aa1n02x5               g231(.a(new_n326), .b(new_n115), .o1(new_n327));
  norb02aa1n02x5               g232(.a(new_n109), .b(new_n115), .out0(new_n328));
  oaib12aa1n02x5               g233(.a(new_n327), .b(new_n325), .c(new_n328), .out0(\s[8] ));
  xorb03aa1n02x5               g234(.a(new_n122), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule

