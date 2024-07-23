// Benchmark "adder" written by ABC on Thu Jul 18 13:02:55 2024

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
    new_n125, new_n126, new_n127, new_n129, new_n130, new_n131, new_n133,
    new_n134, new_n135, new_n136, new_n137, new_n138, new_n139, new_n140,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n150, new_n151, new_n152, new_n153, new_n154, new_n155, new_n156,
    new_n157, new_n158, new_n159, new_n161, new_n162, new_n163, new_n164,
    new_n165, new_n166, new_n167, new_n169, new_n170, new_n171, new_n172,
    new_n173, new_n174, new_n175, new_n176, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n186, new_n187, new_n188,
    new_n189, new_n190, new_n191, new_n192, new_n193, new_n194, new_n195,
    new_n196, new_n197, new_n199, new_n200, new_n201, new_n202, new_n205,
    new_n206, new_n207, new_n208, new_n209, new_n210, new_n211, new_n212,
    new_n213, new_n214, new_n215, new_n216, new_n218, new_n219, new_n220,
    new_n221, new_n222, new_n223, new_n224, new_n225, new_n226, new_n228,
    new_n229, new_n230, new_n231, new_n232, new_n233, new_n234, new_n236,
    new_n237, new_n238, new_n239, new_n240, new_n241, new_n242, new_n243,
    new_n244, new_n245, new_n246, new_n247, new_n248, new_n250, new_n251,
    new_n252, new_n253, new_n254, new_n255, new_n256, new_n258, new_n259,
    new_n260, new_n261, new_n262, new_n263, new_n264, new_n265, new_n267,
    new_n268, new_n269, new_n270, new_n271, new_n272, new_n273, new_n275,
    new_n276, new_n277, new_n278, new_n279, new_n280, new_n281, new_n283,
    new_n284, new_n285, new_n286, new_n287, new_n288, new_n289, new_n290,
    new_n291, new_n292, new_n294, new_n295, new_n296, new_n297, new_n298,
    new_n299, new_n300, new_n301, new_n302, new_n305, new_n306, new_n307,
    new_n308, new_n309, new_n310, new_n311, new_n312, new_n314, new_n315,
    new_n316, new_n317, new_n318, new_n319, new_n320, new_n323, new_n326,
    new_n328, new_n329, new_n330, new_n332, new_n333;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor042aa1d18x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(new_n97), .o1(new_n98));
  nand42aa1n03x5               g003(.a(\b[3] ), .b(\a[4] ), .o1(new_n99));
  nor022aa1n16x5               g004(.a(\b[3] ), .b(\a[4] ), .o1(new_n100));
  norp02aa1n06x5               g005(.a(\b[2] ), .b(\a[3] ), .o1(new_n101));
  oai012aa1n02x5               g006(.a(new_n99), .b(new_n101), .c(new_n100), .o1(new_n102));
  nanp02aa1n02x5               g007(.a(\b[1] ), .b(\a[2] ), .o1(new_n103));
  nand22aa1n12x5               g008(.a(\b[0] ), .b(\a[1] ), .o1(new_n104));
  nor002aa1n16x5               g009(.a(\b[1] ), .b(\a[2] ), .o1(new_n105));
  oaih12aa1n02x5               g010(.a(new_n103), .b(new_n105), .c(new_n104), .o1(new_n106));
  nanp02aa1n02x5               g011(.a(\b[2] ), .b(\a[3] ), .o1(new_n107));
  nona23aa1n03x5               g012(.a(new_n99), .b(new_n107), .c(new_n101), .d(new_n100), .out0(new_n108));
  oai012aa1n12x5               g013(.a(new_n102), .b(new_n108), .c(new_n106), .o1(new_n109));
  nor022aa1n16x5               g014(.a(\b[5] ), .b(\a[6] ), .o1(new_n110));
  nand22aa1n12x5               g015(.a(\b[5] ), .b(\a[6] ), .o1(new_n111));
  norp02aa1n24x5               g016(.a(\b[4] ), .b(\a[5] ), .o1(new_n112));
  nand02aa1n02x5               g017(.a(\b[4] ), .b(\a[5] ), .o1(new_n113));
  nona23aa1n02x4               g018(.a(new_n113), .b(new_n111), .c(new_n110), .d(new_n112), .out0(new_n114));
  xnrc02aa1n12x5               g019(.a(\b[7] ), .b(\a[8] ), .out0(new_n115));
  xnrc02aa1n12x5               g020(.a(\b[6] ), .b(\a[7] ), .out0(new_n116));
  nor043aa1n06x5               g021(.a(new_n114), .b(new_n115), .c(new_n116), .o1(new_n117));
  nanp02aa1n02x5               g022(.a(\b[7] ), .b(\a[8] ), .o1(new_n118));
  norp02aa1n02x5               g023(.a(\b[6] ), .b(\a[7] ), .o1(new_n119));
  oaih12aa1n02x5               g024(.a(new_n118), .b(new_n115), .c(new_n119), .o1(new_n120));
  aoi012aa1n02x7               g025(.a(new_n110), .b(new_n112), .c(new_n111), .o1(new_n121));
  oai013aa1n03x5               g026(.a(new_n120), .b(new_n121), .c(new_n115), .d(new_n116), .o1(new_n122));
  xorc02aa1n12x5               g027(.a(\a[9] ), .b(\b[8] ), .out0(new_n123));
  aoai13aa1n02x5               g028(.a(new_n123), .b(new_n122), .c(new_n109), .d(new_n117), .o1(new_n124));
  nor042aa1n06x5               g029(.a(\b[9] ), .b(\a[10] ), .o1(new_n125));
  nand02aa1d16x5               g030(.a(\b[9] ), .b(\a[10] ), .o1(new_n126));
  norb02aa1n15x5               g031(.a(new_n126), .b(new_n125), .out0(new_n127));
  xnbna2aa1n03x5               g032(.a(new_n127), .b(new_n124), .c(new_n98), .out0(\s[10] ));
  inv000aa1d42x5               g033(.a(new_n127), .o1(new_n129));
  tech160nm_fiaoi012aa1n05x5   g034(.a(new_n125), .b(new_n97), .c(new_n126), .o1(new_n130));
  aoai13aa1n06x5               g035(.a(new_n130), .b(new_n129), .c(new_n124), .d(new_n98), .o1(new_n131));
  xorb03aa1n02x5               g036(.a(new_n131), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  nor002aa1d32x5               g037(.a(\b[10] ), .b(\a[11] ), .o1(new_n133));
  nand42aa1d28x5               g038(.a(\b[10] ), .b(\a[11] ), .o1(new_n134));
  norb02aa1n02x5               g039(.a(new_n134), .b(new_n133), .out0(new_n135));
  nor002aa1n20x5               g040(.a(\b[11] ), .b(\a[12] ), .o1(new_n136));
  nand42aa1d28x5               g041(.a(\b[11] ), .b(\a[12] ), .o1(new_n137));
  nanb02aa1n02x5               g042(.a(new_n136), .b(new_n137), .out0(new_n138));
  aoai13aa1n02x5               g043(.a(new_n138), .b(new_n133), .c(new_n131), .d(new_n135), .o1(new_n139));
  nona22aa1n02x4               g044(.a(new_n137), .b(new_n136), .c(new_n133), .out0(new_n140));
  aoai13aa1n02x5               g045(.a(new_n139), .b(new_n140), .c(new_n135), .d(new_n131), .o1(\s[12] ));
  aoi012aa1n02x7               g046(.a(new_n122), .b(new_n109), .c(new_n117), .o1(new_n142));
  nano23aa1d15x5               g047(.a(new_n133), .b(new_n136), .c(new_n137), .d(new_n134), .out0(new_n143));
  nand23aa1d12x5               g048(.a(new_n143), .b(new_n123), .c(new_n127), .o1(new_n144));
  nona23aa1n03x5               g049(.a(new_n137), .b(new_n134), .c(new_n133), .d(new_n136), .out0(new_n145));
  oa0012aa1n03x5               g050(.a(new_n137), .b(new_n136), .c(new_n133), .o(new_n146));
  oabi12aa1n02x5               g051(.a(new_n146), .b(new_n130), .c(new_n145), .out0(new_n147));
  oabi12aa1n02x5               g052(.a(new_n147), .b(new_n142), .c(new_n144), .out0(new_n148));
  xorb03aa1n02x5               g053(.a(new_n148), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  inv000aa1d42x5               g054(.a(new_n144), .o1(new_n150));
  aoai13aa1n02x5               g055(.a(new_n150), .b(new_n122), .c(new_n109), .d(new_n117), .o1(new_n151));
  nor022aa1n12x5               g056(.a(\b[12] ), .b(\a[13] ), .o1(new_n152));
  nand42aa1d28x5               g057(.a(\b[12] ), .b(\a[13] ), .o1(new_n153));
  nanb02aa1n02x5               g058(.a(new_n152), .b(new_n153), .out0(new_n154));
  aoib12aa1n02x5               g059(.a(new_n154), .b(new_n151), .c(new_n147), .out0(new_n155));
  nor042aa1n06x5               g060(.a(\b[13] ), .b(\a[14] ), .o1(new_n156));
  nand42aa1d28x5               g061(.a(\b[13] ), .b(\a[14] ), .o1(new_n157));
  obai22aa1n02x7               g062(.a(new_n157), .b(new_n156), .c(new_n155), .d(new_n152), .out0(new_n158));
  norb03aa1n02x5               g063(.a(new_n157), .b(new_n152), .c(new_n156), .out0(new_n159));
  oaib12aa1n02x5               g064(.a(new_n158), .b(new_n155), .c(new_n159), .out0(\s[14] ));
  nano23aa1d15x5               g065(.a(new_n152), .b(new_n156), .c(new_n157), .d(new_n153), .out0(new_n161));
  inv000aa1d42x5               g066(.a(new_n161), .o1(new_n162));
  inv000aa1n02x5               g067(.a(new_n130), .o1(new_n163));
  aoai13aa1n06x5               g068(.a(new_n161), .b(new_n146), .c(new_n143), .d(new_n163), .o1(new_n164));
  oai012aa1n12x5               g069(.a(new_n157), .b(new_n156), .c(new_n152), .o1(new_n165));
  nanp02aa1n02x5               g070(.a(new_n164), .b(new_n165), .o1(new_n166));
  oabi12aa1n02x5               g071(.a(new_n166), .b(new_n151), .c(new_n162), .out0(new_n167));
  xorb03aa1n02x5               g072(.a(new_n167), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nor002aa1d32x5               g073(.a(\b[14] ), .b(\a[15] ), .o1(new_n169));
  nand22aa1n12x5               g074(.a(\b[14] ), .b(\a[15] ), .o1(new_n170));
  nanb02aa1d30x5               g075(.a(new_n169), .b(new_n170), .out0(new_n171));
  norb02aa1n02x5               g076(.a(new_n167), .b(new_n171), .out0(new_n172));
  and002aa1n02x5               g077(.a(\b[15] ), .b(\a[16] ), .o(new_n173));
  xnrc02aa1n12x5               g078(.a(\b[15] ), .b(\a[16] ), .out0(new_n174));
  aoai13aa1n02x5               g079(.a(new_n174), .b(new_n169), .c(new_n167), .d(new_n170), .o1(new_n175));
  oai022aa1n02x5               g080(.a(\a[15] ), .b(\b[14] ), .c(\b[15] ), .d(\a[16] ), .o1(new_n176));
  oai013aa1n02x4               g081(.a(new_n175), .b(new_n172), .c(new_n173), .d(new_n176), .o1(\s[16] ));
  nor042aa1n09x5               g082(.a(new_n174), .b(new_n171), .o1(new_n178));
  nano22aa1d15x5               g083(.a(new_n144), .b(new_n178), .c(new_n161), .out0(new_n179));
  aoai13aa1n12x5               g084(.a(new_n179), .b(new_n122), .c(new_n109), .d(new_n117), .o1(new_n180));
  inv000aa1d42x5               g085(.a(new_n165), .o1(new_n181));
  aoai13aa1n03x5               g086(.a(new_n178), .b(new_n181), .c(new_n147), .d(new_n161), .o1(new_n182));
  aob012aa1n02x5               g087(.a(new_n176), .b(\b[15] ), .c(\a[16] ), .out0(new_n183));
  nand23aa1d12x5               g088(.a(new_n180), .b(new_n182), .c(new_n183), .o1(new_n184));
  xorb03aa1n02x5               g089(.a(new_n184), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv000aa1d42x5               g090(.a(\b[16] ), .o1(new_n186));
  nanb02aa1n02x5               g091(.a(\a[17] ), .b(new_n186), .out0(new_n187));
  inv000aa1d42x5               g092(.a(new_n180), .o1(new_n188));
  inv000aa1d42x5               g093(.a(new_n178), .o1(new_n189));
  aoai13aa1n09x5               g094(.a(new_n183), .b(new_n189), .c(new_n164), .d(new_n165), .o1(new_n190));
  xnrc02aa1n02x5               g095(.a(\b[16] ), .b(\a[17] ), .out0(new_n191));
  oabi12aa1n02x5               g096(.a(new_n191), .b(new_n188), .c(new_n190), .out0(new_n192));
  tech160nm_fixorc02aa1n04x5   g097(.a(\a[18] ), .b(\b[17] ), .out0(new_n193));
  inv000aa1n02x5               g098(.a(new_n190), .o1(new_n194));
  oai022aa1n02x5               g099(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n195));
  aoi012aa1n02x5               g100(.a(new_n195), .b(\a[18] ), .c(\b[17] ), .o1(new_n196));
  aoai13aa1n02x5               g101(.a(new_n196), .b(new_n191), .c(new_n194), .d(new_n180), .o1(new_n197));
  aoai13aa1n03x5               g102(.a(new_n197), .b(new_n193), .c(new_n192), .d(new_n187), .o1(\s[18] ));
  inv000aa1d42x5               g103(.a(\b[17] ), .o1(new_n199));
  xroi22aa1d04x5               g104(.a(new_n199), .b(\a[18] ), .c(new_n186), .d(\a[17] ), .out0(new_n200));
  oaoi03aa1n02x5               g105(.a(\a[18] ), .b(\b[17] ), .c(new_n187), .o1(new_n201));
  tech160nm_fiaoi012aa1n05x5   g106(.a(new_n201), .b(new_n184), .c(new_n200), .o1(new_n202));
  xnrb03aa1n03x5               g107(.a(new_n202), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g108(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  inv000aa1d42x5               g109(.a(new_n200), .o1(new_n205));
  oaib12aa1n02x5               g110(.a(new_n195), .b(new_n199), .c(\a[18] ), .out0(new_n206));
  aoai13aa1n02x7               g111(.a(new_n206), .b(new_n205), .c(new_n194), .d(new_n180), .o1(new_n207));
  norp02aa1n06x5               g112(.a(\b[18] ), .b(\a[19] ), .o1(new_n208));
  nand42aa1n06x5               g113(.a(\b[18] ), .b(\a[19] ), .o1(new_n209));
  nor002aa1n06x5               g114(.a(\b[19] ), .b(\a[20] ), .o1(new_n210));
  nand42aa1n08x5               g115(.a(\b[19] ), .b(\a[20] ), .o1(new_n211));
  nanb02aa1n02x5               g116(.a(new_n210), .b(new_n211), .out0(new_n212));
  aoai13aa1n02x5               g117(.a(new_n212), .b(new_n208), .c(new_n207), .d(new_n209), .o1(new_n213));
  nanb02aa1n02x5               g118(.a(new_n208), .b(new_n209), .out0(new_n214));
  norb03aa1n02x5               g119(.a(new_n211), .b(new_n208), .c(new_n210), .out0(new_n215));
  oai012aa1n03x5               g120(.a(new_n215), .b(new_n202), .c(new_n214), .o1(new_n216));
  nanp02aa1n03x5               g121(.a(new_n213), .b(new_n216), .o1(\s[20] ));
  nano23aa1n06x5               g122(.a(new_n208), .b(new_n210), .c(new_n211), .d(new_n209), .out0(new_n218));
  nanb03aa1d18x5               g123(.a(new_n191), .b(new_n218), .c(new_n193), .out0(new_n219));
  inv000aa1d42x5               g124(.a(new_n219), .o1(new_n220));
  nona23aa1n03x5               g125(.a(new_n211), .b(new_n209), .c(new_n208), .d(new_n210), .out0(new_n221));
  tech160nm_fioai012aa1n05x5   g126(.a(new_n211), .b(new_n210), .c(new_n208), .o1(new_n222));
  tech160nm_fioai012aa1n05x5   g127(.a(new_n222), .b(new_n221), .c(new_n206), .o1(new_n223));
  xorc02aa1n02x5               g128(.a(\a[21] ), .b(\b[20] ), .out0(new_n224));
  aoai13aa1n06x5               g129(.a(new_n224), .b(new_n223), .c(new_n184), .d(new_n220), .o1(new_n225));
  aoi112aa1n02x5               g130(.a(new_n224), .b(new_n223), .c(new_n184), .d(new_n220), .o1(new_n226));
  norb02aa1n03x4               g131(.a(new_n225), .b(new_n226), .out0(\s[21] ));
  nor042aa1n06x5               g132(.a(\b[20] ), .b(\a[21] ), .o1(new_n228));
  inv000aa1d42x5               g133(.a(new_n228), .o1(new_n229));
  xorc02aa1n02x5               g134(.a(\a[22] ), .b(\b[21] ), .out0(new_n230));
  nanp02aa1n02x5               g135(.a(\b[21] ), .b(\a[22] ), .o1(new_n231));
  oai022aa1n02x5               g136(.a(\a[21] ), .b(\b[20] ), .c(\b[21] ), .d(\a[22] ), .o1(new_n232));
  norb02aa1n02x5               g137(.a(new_n231), .b(new_n232), .out0(new_n233));
  tech160nm_finand02aa1n03p5x5 g138(.a(new_n225), .b(new_n233), .o1(new_n234));
  aoai13aa1n03x5               g139(.a(new_n234), .b(new_n230), .c(new_n229), .d(new_n225), .o1(\s[22] ));
  nanp02aa1n02x5               g140(.a(\b[20] ), .b(\a[21] ), .o1(new_n236));
  norp02aa1n02x5               g141(.a(\b[21] ), .b(\a[22] ), .o1(new_n237));
  nano23aa1n03x7               g142(.a(new_n228), .b(new_n237), .c(new_n231), .d(new_n236), .out0(new_n238));
  nano23aa1n02x4               g143(.a(new_n191), .b(new_n221), .c(new_n238), .d(new_n193), .out0(new_n239));
  inv000aa1n02x5               g144(.a(new_n222), .o1(new_n240));
  aoai13aa1n06x5               g145(.a(new_n238), .b(new_n240), .c(new_n218), .d(new_n201), .o1(new_n241));
  oaoi03aa1n12x5               g146(.a(\a[22] ), .b(\b[21] ), .c(new_n229), .o1(new_n242));
  inv000aa1d42x5               g147(.a(new_n242), .o1(new_n243));
  nanp02aa1n02x5               g148(.a(new_n241), .b(new_n243), .o1(new_n244));
  xnrc02aa1n12x5               g149(.a(\b[22] ), .b(\a[23] ), .out0(new_n245));
  inv000aa1d42x5               g150(.a(new_n245), .o1(new_n246));
  aoai13aa1n06x5               g151(.a(new_n246), .b(new_n244), .c(new_n184), .d(new_n239), .o1(new_n247));
  aoi112aa1n02x5               g152(.a(new_n246), .b(new_n244), .c(new_n184), .d(new_n239), .o1(new_n248));
  norb02aa1n02x7               g153(.a(new_n247), .b(new_n248), .out0(\s[23] ));
  nor042aa1n02x5               g154(.a(\b[22] ), .b(\a[23] ), .o1(new_n250));
  inv000aa1d42x5               g155(.a(new_n250), .o1(new_n251));
  tech160nm_fixorc02aa1n02p5x5 g156(.a(\a[24] ), .b(\b[23] ), .out0(new_n252));
  inv000aa1d42x5               g157(.a(\a[24] ), .o1(new_n253));
  inv000aa1d42x5               g158(.a(\b[23] ), .o1(new_n254));
  aoi012aa1n02x5               g159(.a(new_n250), .b(new_n253), .c(new_n254), .o1(new_n255));
  oai112aa1n02x7               g160(.a(new_n247), .b(new_n255), .c(new_n254), .d(new_n253), .o1(new_n256));
  aoai13aa1n03x5               g161(.a(new_n256), .b(new_n252), .c(new_n251), .d(new_n247), .o1(\s[24] ));
  nano32aa1n06x5               g162(.a(new_n219), .b(new_n252), .c(new_n238), .d(new_n246), .out0(new_n258));
  norb02aa1n03x5               g163(.a(new_n252), .b(new_n245), .out0(new_n259));
  inv000aa1n02x5               g164(.a(new_n259), .o1(new_n260));
  oaoi03aa1n02x5               g165(.a(new_n253), .b(new_n254), .c(new_n250), .o1(new_n261));
  aoai13aa1n04x5               g166(.a(new_n261), .b(new_n260), .c(new_n241), .d(new_n243), .o1(new_n262));
  tech160nm_fixorc02aa1n03p5x5 g167(.a(\a[25] ), .b(\b[24] ), .out0(new_n263));
  aoai13aa1n06x5               g168(.a(new_n263), .b(new_n262), .c(new_n184), .d(new_n258), .o1(new_n264));
  aoi112aa1n02x5               g169(.a(new_n263), .b(new_n262), .c(new_n184), .d(new_n258), .o1(new_n265));
  norb02aa1n02x7               g170(.a(new_n264), .b(new_n265), .out0(\s[25] ));
  nor042aa1n02x5               g171(.a(\b[24] ), .b(\a[25] ), .o1(new_n267));
  inv000aa1d42x5               g172(.a(new_n267), .o1(new_n268));
  tech160nm_fixorc02aa1n03p5x5 g173(.a(\a[26] ), .b(\b[25] ), .out0(new_n269));
  inv000aa1d42x5               g174(.a(\a[26] ), .o1(new_n270));
  inv000aa1d42x5               g175(.a(\b[25] ), .o1(new_n271));
  aoi012aa1n02x5               g176(.a(new_n267), .b(new_n270), .c(new_n271), .o1(new_n272));
  oai112aa1n02x7               g177(.a(new_n264), .b(new_n272), .c(new_n271), .d(new_n270), .o1(new_n273));
  aoai13aa1n03x5               g178(.a(new_n273), .b(new_n269), .c(new_n268), .d(new_n264), .o1(\s[26] ));
  inv000aa1n02x5               g179(.a(new_n142), .o1(new_n275));
  and002aa1n02x7               g180(.a(new_n269), .b(new_n263), .o(new_n276));
  nano32aa1n03x7               g181(.a(new_n219), .b(new_n276), .c(new_n238), .d(new_n259), .out0(new_n277));
  aoai13aa1n06x5               g182(.a(new_n277), .b(new_n190), .c(new_n275), .d(new_n179), .o1(new_n278));
  oaoi03aa1n02x5               g183(.a(new_n270), .b(new_n271), .c(new_n267), .o1(new_n279));
  aobi12aa1n06x5               g184(.a(new_n279), .b(new_n262), .c(new_n276), .out0(new_n280));
  xorc02aa1n02x5               g185(.a(\a[27] ), .b(\b[26] ), .out0(new_n281));
  xnbna2aa1n03x5               g186(.a(new_n281), .b(new_n278), .c(new_n280), .out0(\s[27] ));
  norp02aa1n02x5               g187(.a(\b[26] ), .b(\a[27] ), .o1(new_n283));
  inv000aa1d42x5               g188(.a(new_n283), .o1(new_n284));
  aoai13aa1n02x5               g189(.a(new_n259), .b(new_n242), .c(new_n223), .d(new_n238), .o1(new_n285));
  inv000aa1d42x5               g190(.a(new_n276), .o1(new_n286));
  aoai13aa1n02x5               g191(.a(new_n279), .b(new_n286), .c(new_n285), .d(new_n261), .o1(new_n287));
  aoai13aa1n03x5               g192(.a(new_n281), .b(new_n287), .c(new_n184), .d(new_n277), .o1(new_n288));
  xnrc02aa1n02x5               g193(.a(\b[27] ), .b(\a[28] ), .out0(new_n289));
  aoi012aa1n03x5               g194(.a(new_n289), .b(new_n288), .c(new_n284), .o1(new_n290));
  aobi12aa1n02x5               g195(.a(new_n281), .b(new_n278), .c(new_n280), .out0(new_n291));
  nano22aa1n02x4               g196(.a(new_n291), .b(new_n284), .c(new_n289), .out0(new_n292));
  nor002aa1n02x5               g197(.a(new_n290), .b(new_n292), .o1(\s[28] ));
  norb02aa1n02x5               g198(.a(new_n281), .b(new_n289), .out0(new_n294));
  aoai13aa1n03x5               g199(.a(new_n294), .b(new_n287), .c(new_n184), .d(new_n277), .o1(new_n295));
  xorc02aa1n02x5               g200(.a(\a[29] ), .b(\b[28] ), .out0(new_n296));
  inv000aa1d42x5               g201(.a(\a[28] ), .o1(new_n297));
  inv000aa1d42x5               g202(.a(\b[27] ), .o1(new_n298));
  aoi112aa1n02x5               g203(.a(\b[26] ), .b(\a[27] ), .c(\a[28] ), .d(\b[27] ), .o1(new_n299));
  aoi112aa1n02x5               g204(.a(new_n296), .b(new_n299), .c(new_n297), .d(new_n298), .o1(new_n300));
  aoi012aa1n02x5               g205(.a(new_n299), .b(new_n297), .c(new_n298), .o1(new_n301));
  nand42aa1n02x5               g206(.a(new_n295), .b(new_n301), .o1(new_n302));
  aoi022aa1n02x7               g207(.a(new_n302), .b(new_n296), .c(new_n295), .d(new_n300), .o1(\s[29] ));
  xorb03aa1n02x5               g208(.a(new_n104), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n02x4               g209(.a(new_n289), .b(new_n281), .c(new_n296), .out0(new_n305));
  aoai13aa1n03x5               g210(.a(new_n305), .b(new_n287), .c(new_n184), .d(new_n277), .o1(new_n306));
  xorc02aa1n02x5               g211(.a(\a[30] ), .b(\b[29] ), .out0(new_n307));
  norp02aa1n02x5               g212(.a(\b[28] ), .b(\a[29] ), .o1(new_n308));
  aoi012aa1n02x5               g213(.a(new_n301), .b(\a[29] ), .c(\b[28] ), .o1(new_n309));
  norp03aa1n02x5               g214(.a(new_n309), .b(new_n307), .c(new_n308), .o1(new_n310));
  oao003aa1n02x5               g215(.a(\a[29] ), .b(\b[28] ), .c(new_n301), .carry(new_n311));
  nand42aa1n02x5               g216(.a(new_n306), .b(new_n311), .o1(new_n312));
  aoi022aa1n02x7               g217(.a(new_n312), .b(new_n307), .c(new_n306), .d(new_n310), .o1(\s[30] ));
  and003aa1n02x5               g218(.a(new_n294), .b(new_n307), .c(new_n296), .o(new_n314));
  aoai13aa1n03x5               g219(.a(new_n314), .b(new_n287), .c(new_n184), .d(new_n277), .o1(new_n315));
  oao003aa1n02x5               g220(.a(\a[30] ), .b(\b[29] ), .c(new_n311), .carry(new_n316));
  xnrc02aa1n02x5               g221(.a(\b[30] ), .b(\a[31] ), .out0(new_n317));
  aoi012aa1n03x5               g222(.a(new_n317), .b(new_n315), .c(new_n316), .o1(new_n318));
  aobi12aa1n02x5               g223(.a(new_n314), .b(new_n278), .c(new_n280), .out0(new_n319));
  nano22aa1n02x4               g224(.a(new_n319), .b(new_n316), .c(new_n317), .out0(new_n320));
  nor002aa1n02x5               g225(.a(new_n318), .b(new_n320), .o1(\s[31] ));
  xnrb03aa1n02x5               g226(.a(new_n106), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g227(.a(\a[3] ), .b(\b[2] ), .c(new_n106), .o1(new_n323));
  xorb03aa1n02x5               g228(.a(new_n323), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g229(.a(new_n109), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  aoi012aa1n02x5               g230(.a(new_n112), .b(new_n109), .c(new_n113), .o1(new_n326));
  xnrb03aa1n02x5               g231(.a(new_n326), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  nanb02aa1n02x5               g232(.a(new_n110), .b(new_n111), .out0(new_n328));
  oaoi13aa1n02x5               g233(.a(new_n116), .b(new_n121), .c(new_n326), .d(new_n328), .o1(new_n329));
  oai112aa1n02x5               g234(.a(new_n121), .b(new_n116), .c(new_n326), .d(new_n328), .o1(new_n330));
  norb02aa1n02x5               g235(.a(new_n330), .b(new_n329), .out0(\s[7] ));
  oabi12aa1n02x5               g236(.a(new_n115), .b(\a[7] ), .c(\b[6] ), .out0(new_n332));
  oai012aa1n02x5               g237(.a(new_n115), .b(new_n329), .c(new_n119), .o1(new_n333));
  oai012aa1n02x5               g238(.a(new_n333), .b(new_n329), .c(new_n332), .o1(\s[8] ));
  xnrc02aa1n02x5               g239(.a(new_n142), .b(new_n123), .out0(\s[9] ));
endmodule


