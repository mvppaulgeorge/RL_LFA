// Benchmark "adder" written by ABC on Thu Jul 18 12:53:56 2024

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
    new_n134, new_n135, new_n136, new_n137, new_n138, new_n139, new_n141,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n153, new_n154, new_n155, new_n156,
    new_n158, new_n159, new_n160, new_n161, new_n162, new_n163, new_n164,
    new_n165, new_n166, new_n168, new_n169, new_n170, new_n171, new_n172,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n184, new_n185, new_n186, new_n187, new_n188,
    new_n189, new_n191, new_n192, new_n193, new_n194, new_n195, new_n196,
    new_n199, new_n200, new_n201, new_n202, new_n203, new_n204, new_n205,
    new_n206, new_n207, new_n208, new_n209, new_n210, new_n211, new_n212,
    new_n214, new_n215, new_n216, new_n217, new_n218, new_n219, new_n221,
    new_n222, new_n223, new_n224, new_n225, new_n226, new_n227, new_n228,
    new_n230, new_n231, new_n232, new_n233, new_n234, new_n235, new_n236,
    new_n237, new_n239, new_n240, new_n241, new_n242, new_n243, new_n244,
    new_n245, new_n247, new_n248, new_n249, new_n250, new_n251, new_n252,
    new_n253, new_n254, new_n255, new_n256, new_n257, new_n258, new_n260,
    new_n261, new_n262, new_n263, new_n264, new_n265, new_n267, new_n268,
    new_n269, new_n270, new_n271, new_n272, new_n273, new_n275, new_n276,
    new_n277, new_n278, new_n279, new_n280, new_n281, new_n282, new_n283,
    new_n284, new_n285, new_n286, new_n287, new_n289, new_n290, new_n291,
    new_n292, new_n293, new_n294, new_n295, new_n298, new_n299, new_n300,
    new_n301, new_n302, new_n303, new_n304, new_n306, new_n307, new_n308,
    new_n309, new_n310, new_n311, new_n313, new_n314, new_n315, new_n316,
    new_n317, new_n319, new_n322, new_n324, new_n325, new_n326, new_n328;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  inv040aa1d32x5               g001(.a(\a[9] ), .o1(new_n97));
  inv040aa1d28x5               g002(.a(\b[8] ), .o1(new_n98));
  nand22aa1n06x5               g003(.a(new_n98), .b(new_n97), .o1(new_n99));
  inv000aa1d42x5               g004(.a(\a[3] ), .o1(new_n100));
  inv000aa1d42x5               g005(.a(\b[2] ), .o1(new_n101));
  nand02aa1n06x5               g006(.a(\b[3] ), .b(\a[4] ), .o1(new_n102));
  nor042aa1n03x5               g007(.a(\b[3] ), .b(\a[4] ), .o1(new_n103));
  aoai13aa1n04x5               g008(.a(new_n102), .b(new_n103), .c(new_n101), .d(new_n100), .o1(new_n104));
  and002aa1n12x5               g009(.a(\b[0] ), .b(\a[1] ), .o(new_n105));
  oaoi03aa1n09x5               g010(.a(\a[2] ), .b(\b[1] ), .c(new_n105), .o1(new_n106));
  xorc02aa1n12x5               g011(.a(\a[3] ), .b(\b[2] ), .out0(new_n107));
  norb02aa1n03x5               g012(.a(new_n102), .b(new_n103), .out0(new_n108));
  nanp03aa1d12x5               g013(.a(new_n106), .b(new_n107), .c(new_n108), .o1(new_n109));
  nor002aa1n06x5               g014(.a(\b[7] ), .b(\a[8] ), .o1(new_n110));
  nanp02aa1n24x5               g015(.a(\b[7] ), .b(\a[8] ), .o1(new_n111));
  nor042aa1n06x5               g016(.a(\b[6] ), .b(\a[7] ), .o1(new_n112));
  nand42aa1n10x5               g017(.a(\b[6] ), .b(\a[7] ), .o1(new_n113));
  nano23aa1n09x5               g018(.a(new_n110), .b(new_n112), .c(new_n113), .d(new_n111), .out0(new_n114));
  tech160nm_fixorc02aa1n05x5   g019(.a(\a[6] ), .b(\b[5] ), .out0(new_n115));
  xorc02aa1n12x5               g020(.a(\a[5] ), .b(\b[4] ), .out0(new_n116));
  nand23aa1n02x5               g021(.a(new_n114), .b(new_n115), .c(new_n116), .o1(new_n117));
  oai022aa1d24x5               g022(.a(\a[7] ), .b(\b[6] ), .c(\b[7] ), .d(\a[8] ), .o1(new_n118));
  orn002aa1n24x5               g023(.a(\a[5] ), .b(\b[4] ), .o(new_n119));
  oaoi03aa1n12x5               g024(.a(\a[6] ), .b(\b[5] ), .c(new_n119), .o1(new_n120));
  aoi022aa1n09x5               g025(.a(new_n114), .b(new_n120), .c(new_n111), .d(new_n118), .o1(new_n121));
  aoai13aa1n12x5               g026(.a(new_n121), .b(new_n117), .c(new_n109), .d(new_n104), .o1(new_n122));
  tech160nm_fixorc02aa1n04x5   g027(.a(\a[9] ), .b(\b[8] ), .out0(new_n123));
  nanp02aa1n06x5               g028(.a(new_n122), .b(new_n123), .o1(new_n124));
  nor042aa1n06x5               g029(.a(\b[9] ), .b(\a[10] ), .o1(new_n125));
  nand42aa1d28x5               g030(.a(\b[9] ), .b(\a[10] ), .o1(new_n126));
  norb02aa1n15x5               g031(.a(new_n126), .b(new_n125), .out0(new_n127));
  xnbna2aa1n03x5               g032(.a(new_n127), .b(new_n124), .c(new_n99), .out0(\s[10] ));
  inv000aa1d42x5               g033(.a(new_n127), .o1(new_n129));
  aoai13aa1n04x5               g034(.a(new_n126), .b(new_n125), .c(new_n97), .d(new_n98), .o1(new_n130));
  aoai13aa1n06x5               g035(.a(new_n130), .b(new_n129), .c(new_n124), .d(new_n99), .o1(new_n131));
  xorb03aa1n02x5               g036(.a(new_n131), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  nor042aa1n12x5               g037(.a(\b[10] ), .b(\a[11] ), .o1(new_n133));
  nand02aa1n12x5               g038(.a(\b[10] ), .b(\a[11] ), .o1(new_n134));
  nor022aa1n08x5               g039(.a(\b[11] ), .b(\a[12] ), .o1(new_n135));
  nand02aa1n10x5               g040(.a(\b[11] ), .b(\a[12] ), .o1(new_n136));
  nanb02aa1n02x5               g041(.a(new_n135), .b(new_n136), .out0(new_n137));
  aoai13aa1n02x7               g042(.a(new_n137), .b(new_n133), .c(new_n131), .d(new_n134), .o1(new_n138));
  aoi112aa1n03x5               g043(.a(new_n133), .b(new_n137), .c(new_n131), .d(new_n134), .o1(new_n139));
  nanb02aa1n03x5               g044(.a(new_n139), .b(new_n138), .out0(\s[12] ));
  nand02aa1n02x5               g045(.a(new_n109), .b(new_n104), .o1(new_n141));
  nona23aa1n02x4               g046(.a(new_n113), .b(new_n111), .c(new_n110), .d(new_n112), .out0(new_n142));
  xnrc02aa1n02x5               g047(.a(\b[4] ), .b(\a[5] ), .out0(new_n143));
  norb03aa1n03x5               g048(.a(new_n115), .b(new_n142), .c(new_n143), .out0(new_n144));
  aobi12aa1n06x5               g049(.a(new_n121), .b(new_n141), .c(new_n144), .out0(new_n145));
  nano23aa1n06x5               g050(.a(new_n133), .b(new_n135), .c(new_n136), .d(new_n134), .out0(new_n146));
  nand03aa1n02x5               g051(.a(new_n146), .b(new_n123), .c(new_n127), .o1(new_n147));
  nona23aa1n03x5               g052(.a(new_n136), .b(new_n134), .c(new_n133), .d(new_n135), .out0(new_n148));
  tech160nm_fiao0012aa1n02p5x5 g053(.a(new_n135), .b(new_n133), .c(new_n136), .o(new_n149));
  oabi12aa1n02x7               g054(.a(new_n149), .b(new_n148), .c(new_n130), .out0(new_n150));
  oabi12aa1n03x5               g055(.a(new_n150), .b(new_n145), .c(new_n147), .out0(new_n151));
  xorb03aa1n02x5               g056(.a(new_n151), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  inv000aa1d42x5               g057(.a(\a[14] ), .o1(new_n153));
  nor042aa1n03x5               g058(.a(\b[12] ), .b(\a[13] ), .o1(new_n154));
  xnrc02aa1n12x5               g059(.a(\b[12] ), .b(\a[13] ), .out0(new_n155));
  aoib12aa1n06x5               g060(.a(new_n154), .b(new_n151), .c(new_n155), .out0(new_n156));
  xorb03aa1n02x5               g061(.a(new_n156), .b(\b[13] ), .c(new_n153), .out0(\s[14] ));
  tech160nm_fixnrc02aa1n05x5   g062(.a(\b[13] ), .b(\a[14] ), .out0(new_n158));
  nona32aa1n03x5               g063(.a(new_n122), .b(new_n158), .c(new_n155), .d(new_n147), .out0(new_n159));
  oaoi03aa1n02x5               g064(.a(\a[10] ), .b(\b[9] ), .c(new_n99), .o1(new_n160));
  nor042aa1n03x5               g065(.a(new_n158), .b(new_n155), .o1(new_n161));
  aoai13aa1n04x5               g066(.a(new_n161), .b(new_n149), .c(new_n146), .d(new_n160), .o1(new_n162));
  inv000aa1d42x5               g067(.a(\b[13] ), .o1(new_n163));
  oao003aa1n02x5               g068(.a(new_n153), .b(new_n163), .c(new_n154), .carry(new_n164));
  inv000aa1n02x5               g069(.a(new_n164), .o1(new_n165));
  nanp03aa1n03x5               g070(.a(new_n159), .b(new_n162), .c(new_n165), .o1(new_n166));
  xorb03aa1n02x5               g071(.a(new_n166), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  inv000aa1d42x5               g072(.a(\a[16] ), .o1(new_n168));
  nor002aa1n02x5               g073(.a(\b[14] ), .b(\a[15] ), .o1(new_n169));
  xnrc02aa1n12x5               g074(.a(\b[14] ), .b(\a[15] ), .out0(new_n170));
  inv000aa1d42x5               g075(.a(new_n170), .o1(new_n171));
  tech160nm_fiaoi012aa1n05x5   g076(.a(new_n169), .b(new_n166), .c(new_n171), .o1(new_n172));
  xorb03aa1n02x5               g077(.a(new_n172), .b(\b[15] ), .c(new_n168), .out0(\s[16] ));
  inv000aa1d42x5               g078(.a(\a[17] ), .o1(new_n174));
  tech160nm_fixnrc02aa1n04x5   g079(.a(\b[15] ), .b(\a[16] ), .out0(new_n175));
  nor042aa1d18x5               g080(.a(new_n175), .b(new_n170), .o1(new_n176));
  nano22aa1n09x5               g081(.a(new_n147), .b(new_n161), .c(new_n176), .out0(new_n177));
  inv040aa1n03x5               g082(.a(new_n176), .o1(new_n178));
  tech160nm_fiaoi012aa1n02p5x5 g083(.a(new_n178), .b(new_n162), .c(new_n165), .o1(new_n179));
  inv000aa1d42x5               g084(.a(\b[15] ), .o1(new_n180));
  oao003aa1n02x5               g085(.a(new_n168), .b(new_n180), .c(new_n169), .carry(new_n181));
  aoi112aa1n09x5               g086(.a(new_n179), .b(new_n181), .c(new_n122), .d(new_n177), .o1(new_n182));
  xorb03aa1n03x5               g087(.a(new_n182), .b(\b[16] ), .c(new_n174), .out0(\s[17] ));
  nanb02aa1n02x5               g088(.a(\b[16] ), .b(new_n174), .out0(new_n184));
  inv000aa1n02x5               g089(.a(new_n181), .o1(new_n185));
  aoai13aa1n03x5               g090(.a(new_n185), .b(new_n178), .c(new_n162), .d(new_n165), .o1(new_n186));
  xorc02aa1n12x5               g091(.a(\a[17] ), .b(\b[16] ), .out0(new_n187));
  aoai13aa1n03x5               g092(.a(new_n187), .b(new_n186), .c(new_n122), .d(new_n177), .o1(new_n188));
  tech160nm_fixnrc02aa1n02p5x5 g093(.a(\b[17] ), .b(\a[18] ), .out0(new_n189));
  xobna2aa1n03x5               g094(.a(new_n189), .b(new_n188), .c(new_n184), .out0(\s[18] ));
  inv000aa1d42x5               g095(.a(\a[18] ), .o1(new_n191));
  xroi22aa1d04x5               g096(.a(new_n174), .b(\b[16] ), .c(new_n191), .d(\b[17] ), .out0(new_n192));
  inv000aa1n02x5               g097(.a(new_n192), .o1(new_n193));
  oaih22aa1d12x5               g098(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n194));
  oaib12aa1n18x5               g099(.a(new_n194), .b(new_n191), .c(\b[17] ), .out0(new_n195));
  tech160nm_fioai012aa1n05x5   g100(.a(new_n195), .b(new_n182), .c(new_n193), .o1(new_n196));
  xorb03aa1n02x5               g101(.a(new_n196), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g102(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor002aa1d32x5               g103(.a(\b[18] ), .b(\a[19] ), .o1(new_n199));
  nand42aa1d28x5               g104(.a(\b[18] ), .b(\a[19] ), .o1(new_n200));
  nanb02aa1d36x5               g105(.a(new_n199), .b(new_n200), .out0(new_n201));
  inv000aa1d42x5               g106(.a(new_n201), .o1(new_n202));
  nor002aa1d32x5               g107(.a(\b[19] ), .b(\a[20] ), .o1(new_n203));
  nand42aa1d28x5               g108(.a(\b[19] ), .b(\a[20] ), .o1(new_n204));
  nanb02aa1n12x5               g109(.a(new_n203), .b(new_n204), .out0(new_n205));
  aoai13aa1n03x5               g110(.a(new_n205), .b(new_n199), .c(new_n196), .d(new_n202), .o1(new_n206));
  nanp02aa1n06x5               g111(.a(new_n122), .b(new_n177), .o1(new_n207));
  aoai13aa1n06x5               g112(.a(new_n176), .b(new_n164), .c(new_n150), .d(new_n161), .o1(new_n208));
  nand23aa1n06x5               g113(.a(new_n207), .b(new_n208), .c(new_n185), .o1(new_n209));
  oaoi03aa1n02x5               g114(.a(\a[18] ), .b(\b[17] ), .c(new_n184), .o1(new_n210));
  aoai13aa1n02x5               g115(.a(new_n202), .b(new_n210), .c(new_n209), .d(new_n192), .o1(new_n211));
  nona22aa1n02x4               g116(.a(new_n211), .b(new_n205), .c(new_n199), .out0(new_n212));
  nanp02aa1n03x5               g117(.a(new_n206), .b(new_n212), .o1(\s[20] ));
  nano23aa1d15x5               g118(.a(new_n199), .b(new_n203), .c(new_n204), .d(new_n200), .out0(new_n214));
  nanb03aa1d24x5               g119(.a(new_n189), .b(new_n214), .c(new_n187), .out0(new_n215));
  aoi012aa1n06x5               g120(.a(new_n203), .b(new_n199), .c(new_n204), .o1(new_n216));
  oai013aa1d12x5               g121(.a(new_n216), .b(new_n195), .c(new_n201), .d(new_n205), .o1(new_n217));
  inv000aa1d42x5               g122(.a(new_n217), .o1(new_n218));
  tech160nm_fioai012aa1n05x5   g123(.a(new_n218), .b(new_n182), .c(new_n215), .o1(new_n219));
  xorb03aa1n02x5               g124(.a(new_n219), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor042aa1d18x5               g125(.a(\b[20] ), .b(\a[21] ), .o1(new_n221));
  xnrc02aa1n12x5               g126(.a(\b[20] ), .b(\a[21] ), .out0(new_n222));
  inv000aa1d42x5               g127(.a(new_n222), .o1(new_n223));
  xnrc02aa1n12x5               g128(.a(\b[21] ), .b(\a[22] ), .out0(new_n224));
  aoai13aa1n03x5               g129(.a(new_n224), .b(new_n221), .c(new_n219), .d(new_n223), .o1(new_n225));
  inv000aa1d42x5               g130(.a(new_n215), .o1(new_n226));
  aoai13aa1n02x5               g131(.a(new_n223), .b(new_n217), .c(new_n209), .d(new_n226), .o1(new_n227));
  nona22aa1n02x4               g132(.a(new_n227), .b(new_n224), .c(new_n221), .out0(new_n228));
  nanp02aa1n03x5               g133(.a(new_n225), .b(new_n228), .o1(\s[22] ));
  nor042aa1n06x5               g134(.a(new_n224), .b(new_n222), .o1(new_n230));
  nano22aa1n03x7               g135(.a(new_n193), .b(new_n230), .c(new_n214), .out0(new_n231));
  inv000aa1n02x5               g136(.a(new_n231), .o1(new_n232));
  inv040aa1n08x5               g137(.a(\a[22] ), .o1(new_n233));
  inv040aa1d32x5               g138(.a(\b[21] ), .o1(new_n234));
  oao003aa1n09x5               g139(.a(new_n233), .b(new_n234), .c(new_n221), .carry(new_n235));
  aoi012aa1d24x5               g140(.a(new_n235), .b(new_n217), .c(new_n230), .o1(new_n236));
  tech160nm_fioai012aa1n05x5   g141(.a(new_n236), .b(new_n182), .c(new_n232), .o1(new_n237));
  xorb03aa1n02x5               g142(.a(new_n237), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g143(.a(\b[22] ), .b(\a[23] ), .o1(new_n239));
  xorc02aa1n12x5               g144(.a(\a[23] ), .b(\b[22] ), .out0(new_n240));
  tech160nm_fixnrc02aa1n05x5   g145(.a(\b[23] ), .b(\a[24] ), .out0(new_n241));
  aoai13aa1n03x5               g146(.a(new_n241), .b(new_n239), .c(new_n237), .d(new_n240), .o1(new_n242));
  inv000aa1d42x5               g147(.a(new_n236), .o1(new_n243));
  aoai13aa1n02x5               g148(.a(new_n240), .b(new_n243), .c(new_n209), .d(new_n231), .o1(new_n244));
  nona22aa1n02x4               g149(.a(new_n244), .b(new_n241), .c(new_n239), .out0(new_n245));
  nanp02aa1n03x5               g150(.a(new_n242), .b(new_n245), .o1(\s[24] ));
  norb02aa1n02x5               g151(.a(new_n240), .b(new_n241), .out0(new_n247));
  inv030aa1n02x5               g152(.a(new_n247), .o1(new_n248));
  nano32aa1n03x7               g153(.a(new_n248), .b(new_n192), .c(new_n230), .d(new_n214), .out0(new_n249));
  inv000aa1n02x5               g154(.a(new_n249), .o1(new_n250));
  inv000aa1n02x5               g155(.a(new_n216), .o1(new_n251));
  aoai13aa1n06x5               g156(.a(new_n230), .b(new_n251), .c(new_n214), .d(new_n210), .o1(new_n252));
  inv000aa1n02x5               g157(.a(new_n235), .o1(new_n253));
  oai022aa1n02x5               g158(.a(\a[23] ), .b(\b[22] ), .c(\b[23] ), .d(\a[24] ), .o1(new_n254));
  aob012aa1n02x5               g159(.a(new_n254), .b(\b[23] ), .c(\a[24] ), .out0(new_n255));
  aoai13aa1n06x5               g160(.a(new_n255), .b(new_n248), .c(new_n252), .d(new_n253), .o1(new_n256));
  inv000aa1n02x5               g161(.a(new_n256), .o1(new_n257));
  tech160nm_fioai012aa1n05x5   g162(.a(new_n257), .b(new_n182), .c(new_n250), .o1(new_n258));
  xorb03aa1n02x5               g163(.a(new_n258), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g164(.a(\b[24] ), .b(\a[25] ), .o1(new_n260));
  xorc02aa1n03x5               g165(.a(\a[25] ), .b(\b[24] ), .out0(new_n261));
  xnrc02aa1n12x5               g166(.a(\b[25] ), .b(\a[26] ), .out0(new_n262));
  aoai13aa1n03x5               g167(.a(new_n262), .b(new_n260), .c(new_n258), .d(new_n261), .o1(new_n263));
  aoai13aa1n02x5               g168(.a(new_n261), .b(new_n256), .c(new_n209), .d(new_n249), .o1(new_n264));
  nona22aa1n02x4               g169(.a(new_n264), .b(new_n262), .c(new_n260), .out0(new_n265));
  nanp02aa1n03x5               g170(.a(new_n263), .b(new_n265), .o1(\s[26] ));
  norb02aa1n06x5               g171(.a(new_n261), .b(new_n262), .out0(new_n267));
  nano23aa1n06x5               g172(.a(new_n215), .b(new_n248), .c(new_n267), .d(new_n230), .out0(new_n268));
  aoai13aa1n06x5               g173(.a(new_n268), .b(new_n186), .c(new_n122), .d(new_n177), .o1(new_n269));
  nanp02aa1n02x5               g174(.a(\b[25] ), .b(\a[26] ), .o1(new_n270));
  oai022aa1n02x5               g175(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n271));
  aoi022aa1n09x5               g176(.a(new_n256), .b(new_n267), .c(new_n270), .d(new_n271), .o1(new_n272));
  xorc02aa1n12x5               g177(.a(\a[27] ), .b(\b[26] ), .out0(new_n273));
  xnbna2aa1n03x5               g178(.a(new_n273), .b(new_n269), .c(new_n272), .out0(\s[27] ));
  inv000aa1n02x5               g179(.a(new_n268), .o1(new_n275));
  tech160nm_fioai012aa1n05x5   g180(.a(new_n272), .b(new_n182), .c(new_n275), .o1(new_n276));
  norp02aa1n02x5               g181(.a(\b[26] ), .b(\a[27] ), .o1(new_n277));
  nor002aa1n03x5               g182(.a(\b[27] ), .b(\a[28] ), .o1(new_n278));
  nanp02aa1n04x5               g183(.a(\b[27] ), .b(\a[28] ), .o1(new_n279));
  nanb02aa1n06x5               g184(.a(new_n278), .b(new_n279), .out0(new_n280));
  aoai13aa1n03x5               g185(.a(new_n280), .b(new_n277), .c(new_n276), .d(new_n273), .o1(new_n281));
  aoai13aa1n03x5               g186(.a(new_n247), .b(new_n235), .c(new_n217), .d(new_n230), .o1(new_n282));
  inv000aa1d42x5               g187(.a(new_n267), .o1(new_n283));
  nanp02aa1n02x5               g188(.a(new_n271), .b(new_n270), .o1(new_n284));
  aoai13aa1n04x5               g189(.a(new_n284), .b(new_n283), .c(new_n282), .d(new_n255), .o1(new_n285));
  aoai13aa1n02x5               g190(.a(new_n273), .b(new_n285), .c(new_n209), .d(new_n268), .o1(new_n286));
  nona22aa1n02x4               g191(.a(new_n286), .b(new_n280), .c(new_n277), .out0(new_n287));
  nanp02aa1n03x5               g192(.a(new_n281), .b(new_n287), .o1(\s[28] ));
  norb02aa1n09x5               g193(.a(new_n273), .b(new_n280), .out0(new_n289));
  aoai13aa1n02x5               g194(.a(new_n289), .b(new_n285), .c(new_n209), .d(new_n268), .o1(new_n290));
  xorc02aa1n12x5               g195(.a(\a[29] ), .b(\b[28] ), .out0(new_n291));
  aoi012aa1n02x5               g196(.a(new_n278), .b(new_n277), .c(new_n279), .o1(new_n292));
  norb02aa1n02x5               g197(.a(new_n292), .b(new_n291), .out0(new_n293));
  inv000aa1d42x5               g198(.a(new_n289), .o1(new_n294));
  aoai13aa1n03x5               g199(.a(new_n292), .b(new_n294), .c(new_n269), .d(new_n272), .o1(new_n295));
  aoi022aa1n03x5               g200(.a(new_n295), .b(new_n291), .c(new_n290), .d(new_n293), .o1(\s[29] ));
  xnrb03aa1n02x5               g201(.a(new_n105), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n02x4               g202(.a(new_n280), .b(new_n273), .c(new_n291), .out0(new_n298));
  aoai13aa1n02x5               g203(.a(new_n298), .b(new_n285), .c(new_n209), .d(new_n268), .o1(new_n299));
  xorc02aa1n02x5               g204(.a(\a[30] ), .b(\b[29] ), .out0(new_n300));
  oao003aa1n02x5               g205(.a(\a[29] ), .b(\b[28] ), .c(new_n292), .carry(new_n301));
  norb02aa1n02x5               g206(.a(new_n301), .b(new_n300), .out0(new_n302));
  inv000aa1n02x5               g207(.a(new_n298), .o1(new_n303));
  aoai13aa1n03x5               g208(.a(new_n301), .b(new_n303), .c(new_n269), .d(new_n272), .o1(new_n304));
  aoi022aa1n03x5               g209(.a(new_n304), .b(new_n300), .c(new_n299), .d(new_n302), .o1(\s[30] ));
  nanp03aa1n02x5               g210(.a(new_n289), .b(new_n291), .c(new_n300), .o1(new_n306));
  nanb02aa1n03x5               g211(.a(new_n306), .b(new_n276), .out0(new_n307));
  xorc02aa1n02x5               g212(.a(\a[31] ), .b(\b[30] ), .out0(new_n308));
  oao003aa1n02x5               g213(.a(\a[30] ), .b(\b[29] ), .c(new_n301), .carry(new_n309));
  norb02aa1n02x5               g214(.a(new_n309), .b(new_n308), .out0(new_n310));
  aoai13aa1n02x7               g215(.a(new_n309), .b(new_n306), .c(new_n269), .d(new_n272), .o1(new_n311));
  aoi022aa1n03x5               g216(.a(new_n311), .b(new_n308), .c(new_n307), .d(new_n310), .o1(\s[31] ));
  inv000aa1d42x5               g217(.a(\a[1] ), .o1(new_n313));
  inv000aa1d42x5               g218(.a(\b[0] ), .o1(new_n314));
  nanp02aa1n02x5               g219(.a(\b[1] ), .b(\a[2] ), .o1(new_n315));
  norp02aa1n02x5               g220(.a(\b[1] ), .b(\a[2] ), .o1(new_n316));
  oai013aa1n02x4               g221(.a(new_n315), .b(new_n316), .c(new_n313), .d(new_n314), .o1(new_n317));
  xorb03aa1n02x5               g222(.a(new_n317), .b(\b[2] ), .c(new_n100), .out0(\s[3] ));
  oaoi03aa1n02x5               g223(.a(\a[3] ), .b(\b[2] ), .c(new_n317), .o1(new_n319));
  xorb03aa1n02x5               g224(.a(new_n319), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xnbna2aa1n03x5               g225(.a(new_n116), .b(new_n109), .c(new_n104), .out0(\s[5] ));
  aoai13aa1n03x5               g226(.a(new_n119), .b(new_n143), .c(new_n109), .d(new_n104), .o1(new_n322));
  xorb03aa1n02x5               g227(.a(new_n322), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  norb02aa1n02x5               g228(.a(new_n113), .b(new_n112), .out0(new_n324));
  aoai13aa1n03x5               g229(.a(new_n324), .b(new_n120), .c(new_n322), .d(new_n115), .o1(new_n325));
  aoi112aa1n02x5               g230(.a(new_n120), .b(new_n324), .c(new_n322), .d(new_n115), .o1(new_n326));
  norb02aa1n02x5               g231(.a(new_n325), .b(new_n326), .out0(\s[7] ));
  tech160nm_fioai012aa1n03p5x5 g232(.a(new_n325), .b(\b[6] ), .c(\a[7] ), .o1(new_n328));
  xorb03aa1n02x5               g233(.a(new_n328), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xorb03aa1n02x5               g234(.a(new_n122), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


