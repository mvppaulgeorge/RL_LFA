// Benchmark "adder" written by ABC on Thu Jul 11 11:46:59 2024

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
    new_n133, new_n134, new_n135, new_n136, new_n138, new_n140, new_n141,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n151, new_n152, new_n153, new_n155, new_n156, new_n157,
    new_n158, new_n159, new_n160, new_n161, new_n163, new_n164, new_n165,
    new_n166, new_n167, new_n168, new_n169, new_n170, new_n172, new_n173,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n185, new_n186, new_n187, new_n188,
    new_n190, new_n191, new_n192, new_n193, new_n194, new_n197, new_n198,
    new_n199, new_n200, new_n201, new_n202, new_n203, new_n205, new_n206,
    new_n207, new_n208, new_n209, new_n210, new_n211, new_n213, new_n214,
    new_n215, new_n216, new_n217, new_n219, new_n220, new_n221, new_n222,
    new_n223, new_n224, new_n225, new_n226, new_n228, new_n229, new_n230,
    new_n231, new_n232, new_n234, new_n235, new_n236, new_n237, new_n238,
    new_n239, new_n240, new_n241, new_n242, new_n243, new_n244, new_n245,
    new_n246, new_n247, new_n248, new_n249, new_n251, new_n252, new_n253,
    new_n254, new_n255, new_n257, new_n258, new_n259, new_n260, new_n261,
    new_n262, new_n263, new_n264, new_n265, new_n266, new_n267, new_n268,
    new_n269, new_n271, new_n272, new_n273, new_n274, new_n275, new_n276,
    new_n277, new_n278, new_n279, new_n280, new_n282, new_n283, new_n284,
    new_n285, new_n286, new_n287, new_n288, new_n291, new_n292, new_n293,
    new_n294, new_n295, new_n296, new_n297, new_n299, new_n300, new_n301,
    new_n302, new_n303, new_n304, new_n305, new_n308, new_n311, new_n313,
    new_n315, new_n316;
  160nm_ficinv00aa1n08x5       g000(.clk(\a[0] ), .clkout(\s[0] ));
  xorc02aa1n02x5               g001(.a(\a[10] ), .b(\b[9] ), .out0(new_n97));
  160nm_ficinv00aa1n08x5       g002(.clk(\a[9] ), .clkout(new_n98));
  nanb02aa1n02x5               g003(.a(\b[8] ), .b(new_n98), .out0(new_n99));
  and002aa1n02x5               g004(.a(\b[3] ), .b(\a[4] ), .o(new_n100));
  160nm_ficinv00aa1n08x5       g005(.clk(\a[3] ), .clkout(new_n101));
  160nm_ficinv00aa1n08x5       g006(.clk(\b[2] ), .clkout(new_n102));
  nanp02aa1n02x5               g007(.a(new_n102), .b(new_n101), .o1(new_n103));
  nanp02aa1n02x5               g008(.a(\b[2] ), .b(\a[3] ), .o1(new_n104));
  nanp02aa1n02x5               g009(.a(new_n103), .b(new_n104), .o1(new_n105));
  norp02aa1n02x5               g010(.a(\b[1] ), .b(\a[2] ), .o1(new_n106));
  nanp02aa1n02x5               g011(.a(\b[1] ), .b(\a[2] ), .o1(new_n107));
  nanp02aa1n02x5               g012(.a(\b[0] ), .b(\a[1] ), .o1(new_n108));
  aoi012aa1n02x5               g013(.a(new_n106), .b(new_n107), .c(new_n108), .o1(new_n109));
  160nm_ficinv00aa1n08x5       g014(.clk(\a[4] ), .clkout(new_n110));
  aboi22aa1n03x5               g015(.a(\b[3] ), .b(new_n110), .c(new_n101), .d(new_n102), .out0(new_n111));
  oaoi13aa1n02x5               g016(.a(new_n100), .b(new_n111), .c(new_n109), .d(new_n105), .o1(new_n112));
  xnrc02aa1n02x5               g017(.a(\b[7] ), .b(\a[8] ), .out0(new_n113));
  xnrc02aa1n02x5               g018(.a(\b[6] ), .b(\a[7] ), .out0(new_n114));
  norp02aa1n02x5               g019(.a(\b[5] ), .b(\a[6] ), .o1(new_n115));
  nanp02aa1n02x5               g020(.a(\b[5] ), .b(\a[6] ), .o1(new_n116));
  norp02aa1n02x5               g021(.a(\b[4] ), .b(\a[5] ), .o1(new_n117));
  nanp02aa1n02x5               g022(.a(\b[4] ), .b(\a[5] ), .o1(new_n118));
  nona23aa1n02x4               g023(.a(new_n118), .b(new_n116), .c(new_n115), .d(new_n117), .out0(new_n119));
  norp03aa1n02x5               g024(.a(new_n119), .b(new_n114), .c(new_n113), .o1(new_n120));
  oai012aa1n02x5               g025(.a(new_n116), .b(new_n117), .c(new_n115), .o1(new_n121));
  160nm_ficinv00aa1n08x5       g026(.clk(\a[8] ), .clkout(new_n122));
  160nm_ficinv00aa1n08x5       g027(.clk(\b[7] ), .clkout(new_n123));
  norp02aa1n02x5               g028(.a(\b[6] ), .b(\a[7] ), .o1(new_n124));
  oaoi03aa1n02x5               g029(.a(new_n122), .b(new_n123), .c(new_n124), .o1(new_n125));
  oai013aa1n02x4               g030(.a(new_n125), .b(new_n114), .c(new_n113), .d(new_n121), .o1(new_n126));
  xorc02aa1n02x5               g031(.a(\a[9] ), .b(\b[8] ), .out0(new_n127));
  aoai13aa1n02x5               g032(.a(new_n127), .b(new_n126), .c(new_n112), .d(new_n120), .o1(new_n128));
  xnbna2aa1n03x5               g033(.a(new_n97), .b(new_n128), .c(new_n99), .out0(\s[10] ));
  oai012aa1n02x5               g034(.a(new_n111), .b(new_n109), .c(new_n105), .o1(new_n130));
  oaib12aa1n02x5               g035(.a(new_n130), .b(new_n110), .c(\b[3] ), .out0(new_n131));
  orn003aa1n02x5               g036(.a(new_n119), .b(new_n113), .c(new_n114), .o(new_n132));
  oabi12aa1n02x5               g037(.a(new_n126), .b(new_n132), .c(new_n131), .out0(new_n133));
  oaoi03aa1n02x5               g038(.a(\a[10] ), .b(\b[9] ), .c(new_n99), .o1(new_n134));
  and002aa1n02x5               g039(.a(new_n127), .b(new_n97), .o(new_n135));
  aoi012aa1n02x5               g040(.a(new_n134), .b(new_n133), .c(new_n135), .o1(new_n136));
  xnrb03aa1n02x5               g041(.a(new_n136), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  oaoi03aa1n02x5               g042(.a(\a[11] ), .b(\b[10] ), .c(new_n136), .o1(new_n138));
  xorb03aa1n02x5               g043(.a(new_n138), .b(\b[11] ), .c(\a[12] ), .out0(\s[12] ));
  norp02aa1n02x5               g044(.a(\b[10] ), .b(\a[11] ), .o1(new_n140));
  nanp02aa1n02x5               g045(.a(\b[10] ), .b(\a[11] ), .o1(new_n141));
  norp02aa1n02x5               g046(.a(\b[11] ), .b(\a[12] ), .o1(new_n142));
  nanp02aa1n02x5               g047(.a(\b[11] ), .b(\a[12] ), .o1(new_n143));
  nano23aa1n02x4               g048(.a(new_n140), .b(new_n142), .c(new_n143), .d(new_n141), .out0(new_n144));
  and003aa1n02x5               g049(.a(new_n144), .b(new_n127), .c(new_n97), .o(new_n145));
  aoai13aa1n02x5               g050(.a(new_n145), .b(new_n126), .c(new_n112), .d(new_n120), .o1(new_n146));
  160nm_fiao0012aa1n02p5x5     g051(.a(new_n142), .b(new_n140), .c(new_n143), .o(new_n147));
  aoi012aa1n02x5               g052(.a(new_n147), .b(new_n144), .c(new_n134), .o1(new_n148));
  nanp02aa1n02x5               g053(.a(new_n146), .b(new_n148), .o1(new_n149));
  xorb03aa1n02x5               g054(.a(new_n149), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  160nm_ficinv00aa1n08x5       g055(.clk(\a[13] ), .clkout(new_n151));
  160nm_ficinv00aa1n08x5       g056(.clk(\b[12] ), .clkout(new_n152));
  oaoi03aa1n02x5               g057(.a(new_n151), .b(new_n152), .c(new_n149), .o1(new_n153));
  xnrb03aa1n02x5               g058(.a(new_n153), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  norp02aa1n02x5               g059(.a(\b[12] ), .b(\a[13] ), .o1(new_n155));
  nanp02aa1n02x5               g060(.a(\b[12] ), .b(\a[13] ), .o1(new_n156));
  norp02aa1n02x5               g061(.a(\b[13] ), .b(\a[14] ), .o1(new_n157));
  nanp02aa1n02x5               g062(.a(\b[13] ), .b(\a[14] ), .o1(new_n158));
  nona23aa1n02x4               g063(.a(new_n158), .b(new_n156), .c(new_n155), .d(new_n157), .out0(new_n159));
  aoai13aa1n02x5               g064(.a(new_n158), .b(new_n157), .c(new_n151), .d(new_n152), .o1(new_n160));
  aoai13aa1n02x5               g065(.a(new_n160), .b(new_n159), .c(new_n146), .d(new_n148), .o1(new_n161));
  xorb03aa1n02x5               g066(.a(new_n161), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  norp02aa1n02x5               g067(.a(\b[14] ), .b(\a[15] ), .o1(new_n163));
  nanp02aa1n02x5               g068(.a(\b[14] ), .b(\a[15] ), .o1(new_n164));
  norb02aa1n02x5               g069(.a(new_n164), .b(new_n163), .out0(new_n165));
  norp02aa1n02x5               g070(.a(\b[15] ), .b(\a[16] ), .o1(new_n166));
  nanp02aa1n02x5               g071(.a(\b[15] ), .b(\a[16] ), .o1(new_n167));
  norb02aa1n02x5               g072(.a(new_n167), .b(new_n166), .out0(new_n168));
  aoi112aa1n02x5               g073(.a(new_n168), .b(new_n163), .c(new_n161), .d(new_n165), .o1(new_n169));
  aoai13aa1n02x5               g074(.a(new_n168), .b(new_n163), .c(new_n161), .d(new_n164), .o1(new_n170));
  norb02aa1n02x5               g075(.a(new_n170), .b(new_n169), .out0(\s[16] ));
  nano23aa1n02x4               g076(.a(new_n155), .b(new_n157), .c(new_n158), .d(new_n156), .out0(new_n172));
  nano23aa1n02x4               g077(.a(new_n163), .b(new_n166), .c(new_n167), .d(new_n164), .out0(new_n173));
  nanp02aa1n02x5               g078(.a(new_n173), .b(new_n172), .o1(new_n174));
  nano32aa1n02x4               g079(.a(new_n174), .b(new_n144), .c(new_n127), .d(new_n97), .out0(new_n175));
  aoai13aa1n02x5               g080(.a(new_n175), .b(new_n126), .c(new_n112), .d(new_n120), .o1(new_n176));
  160nm_ficinv00aa1n08x5       g081(.clk(new_n166), .clkout(new_n177));
  nanb03aa1n02x5               g082(.a(new_n160), .b(new_n168), .c(new_n165), .out0(new_n178));
  aoi112aa1n02x5               g083(.a(\b[14] ), .b(\a[15] ), .c(\a[16] ), .d(\b[15] ), .o1(new_n179));
  160nm_ficinv00aa1n08x5       g084(.clk(new_n179), .clkout(new_n180));
  norp02aa1n02x5               g085(.a(new_n148), .b(new_n174), .o1(new_n181));
  nano32aa1n02x4               g086(.a(new_n181), .b(new_n180), .c(new_n178), .d(new_n177), .out0(new_n182));
  nanp02aa1n02x5               g087(.a(new_n182), .b(new_n176), .o1(new_n183));
  xorb03aa1n02x5               g088(.a(new_n183), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  160nm_ficinv00aa1n08x5       g089(.clk(\a[18] ), .clkout(new_n185));
  160nm_ficinv00aa1n08x5       g090(.clk(\a[17] ), .clkout(new_n186));
  160nm_ficinv00aa1n08x5       g091(.clk(\b[16] ), .clkout(new_n187));
  oaoi03aa1n02x5               g092(.a(new_n186), .b(new_n187), .c(new_n183), .o1(new_n188));
  xorb03aa1n02x5               g093(.a(new_n188), .b(\b[17] ), .c(new_n185), .out0(\s[18] ));
  xroi22aa1d04x5               g094(.a(new_n186), .b(\b[16] ), .c(new_n185), .d(\b[17] ), .out0(new_n190));
  160nm_ficinv00aa1n08x5       g095(.clk(new_n190), .clkout(new_n191));
  oai022aa1n02x5               g096(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n192));
  oaib12aa1n02x5               g097(.a(new_n192), .b(new_n185), .c(\b[17] ), .out0(new_n193));
  aoai13aa1n02x5               g098(.a(new_n193), .b(new_n191), .c(new_n182), .d(new_n176), .o1(new_n194));
  xorb03aa1n02x5               g099(.a(new_n194), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g100(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  norp02aa1n02x5               g101(.a(\b[18] ), .b(\a[19] ), .o1(new_n197));
  nanp02aa1n02x5               g102(.a(\b[18] ), .b(\a[19] ), .o1(new_n198));
  norp02aa1n02x5               g103(.a(\b[19] ), .b(\a[20] ), .o1(new_n199));
  nanp02aa1n02x5               g104(.a(\b[19] ), .b(\a[20] ), .o1(new_n200));
  norb02aa1n02x5               g105(.a(new_n200), .b(new_n199), .out0(new_n201));
  aoi112aa1n02x5               g106(.a(new_n197), .b(new_n201), .c(new_n194), .d(new_n198), .o1(new_n202));
  aoai13aa1n02x5               g107(.a(new_n201), .b(new_n197), .c(new_n194), .d(new_n198), .o1(new_n203));
  norb02aa1n02x5               g108(.a(new_n203), .b(new_n202), .out0(\s[20] ));
  nano23aa1n02x4               g109(.a(new_n197), .b(new_n199), .c(new_n200), .d(new_n198), .out0(new_n205));
  nanp02aa1n02x5               g110(.a(new_n190), .b(new_n205), .o1(new_n206));
  nona23aa1n02x4               g111(.a(new_n200), .b(new_n198), .c(new_n197), .d(new_n199), .out0(new_n207));
  aoi012aa1n02x5               g112(.a(new_n199), .b(new_n197), .c(new_n200), .o1(new_n208));
  oai012aa1n02x5               g113(.a(new_n208), .b(new_n207), .c(new_n193), .o1(new_n209));
  160nm_ficinv00aa1n08x5       g114(.clk(new_n209), .clkout(new_n210));
  aoai13aa1n02x5               g115(.a(new_n210), .b(new_n206), .c(new_n182), .d(new_n176), .o1(new_n211));
  xorb03aa1n02x5               g116(.a(new_n211), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g117(.a(\b[20] ), .b(\a[21] ), .o1(new_n213));
  xorc02aa1n02x5               g118(.a(\a[21] ), .b(\b[20] ), .out0(new_n214));
  xorc02aa1n02x5               g119(.a(\a[22] ), .b(\b[21] ), .out0(new_n215));
  aoi112aa1n02x5               g120(.a(new_n213), .b(new_n215), .c(new_n211), .d(new_n214), .o1(new_n216));
  aoai13aa1n02x5               g121(.a(new_n215), .b(new_n213), .c(new_n211), .d(new_n214), .o1(new_n217));
  norb02aa1n02x5               g122(.a(new_n217), .b(new_n216), .out0(\s[22] ));
  160nm_ficinv00aa1n08x5       g123(.clk(\a[21] ), .clkout(new_n219));
  160nm_ficinv00aa1n08x5       g124(.clk(\a[22] ), .clkout(new_n220));
  xroi22aa1d04x5               g125(.a(new_n219), .b(\b[20] ), .c(new_n220), .d(\b[21] ), .out0(new_n221));
  nanp03aa1n02x5               g126(.a(new_n221), .b(new_n190), .c(new_n205), .o1(new_n222));
  160nm_ficinv00aa1n08x5       g127(.clk(\b[21] ), .clkout(new_n223));
  oao003aa1n02x5               g128(.a(new_n220), .b(new_n223), .c(new_n213), .carry(new_n224));
  aoi012aa1n02x5               g129(.a(new_n224), .b(new_n209), .c(new_n221), .o1(new_n225));
  aoai13aa1n02x5               g130(.a(new_n225), .b(new_n222), .c(new_n182), .d(new_n176), .o1(new_n226));
  xorb03aa1n02x5               g131(.a(new_n226), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g132(.a(\b[22] ), .b(\a[23] ), .o1(new_n228));
  xorc02aa1n02x5               g133(.a(\a[23] ), .b(\b[22] ), .out0(new_n229));
  xorc02aa1n02x5               g134(.a(\a[24] ), .b(\b[23] ), .out0(new_n230));
  aoi112aa1n02x5               g135(.a(new_n228), .b(new_n230), .c(new_n226), .d(new_n229), .o1(new_n231));
  aoai13aa1n02x5               g136(.a(new_n230), .b(new_n228), .c(new_n226), .d(new_n229), .o1(new_n232));
  norb02aa1n02x5               g137(.a(new_n232), .b(new_n231), .out0(\s[24] ));
  160nm_ficinv00aa1n08x5       g138(.clk(\a[23] ), .clkout(new_n234));
  160nm_ficinv00aa1n08x5       g139(.clk(\a[24] ), .clkout(new_n235));
  xroi22aa1d04x5               g140(.a(new_n234), .b(\b[22] ), .c(new_n235), .d(\b[23] ), .out0(new_n236));
  nano22aa1n02x4               g141(.a(new_n206), .b(new_n221), .c(new_n236), .out0(new_n237));
  nanp02aa1n02x5               g142(.a(new_n187), .b(new_n186), .o1(new_n238));
  oaoi03aa1n02x5               g143(.a(\a[18] ), .b(\b[17] ), .c(new_n238), .o1(new_n239));
  160nm_ficinv00aa1n08x5       g144(.clk(new_n208), .clkout(new_n240));
  aoai13aa1n02x5               g145(.a(new_n221), .b(new_n240), .c(new_n205), .d(new_n239), .o1(new_n241));
  160nm_ficinv00aa1n08x5       g146(.clk(new_n224), .clkout(new_n242));
  160nm_ficinv00aa1n08x5       g147(.clk(new_n236), .clkout(new_n243));
  aoi112aa1n02x5               g148(.a(\b[22] ), .b(\a[23] ), .c(\a[24] ), .d(\b[23] ), .o1(new_n244));
  aoib12aa1n02x5               g149(.a(new_n244), .b(new_n235), .c(\b[23] ), .out0(new_n245));
  aoai13aa1n02x5               g150(.a(new_n245), .b(new_n243), .c(new_n241), .d(new_n242), .o1(new_n246));
  xorc02aa1n02x5               g151(.a(\a[25] ), .b(\b[24] ), .out0(new_n247));
  aoai13aa1n02x5               g152(.a(new_n247), .b(new_n246), .c(new_n183), .d(new_n237), .o1(new_n248));
  aoi112aa1n02x5               g153(.a(new_n247), .b(new_n246), .c(new_n183), .d(new_n237), .o1(new_n249));
  norb02aa1n02x5               g154(.a(new_n248), .b(new_n249), .out0(\s[25] ));
  norp02aa1n02x5               g155(.a(\b[24] ), .b(\a[25] ), .o1(new_n251));
  xorc02aa1n02x5               g156(.a(\a[26] ), .b(\b[25] ), .out0(new_n252));
  nona22aa1n02x4               g157(.a(new_n248), .b(new_n252), .c(new_n251), .out0(new_n253));
  160nm_ficinv00aa1n08x5       g158(.clk(new_n251), .clkout(new_n254));
  aobi12aa1n02x5               g159(.a(new_n252), .b(new_n248), .c(new_n254), .out0(new_n255));
  norb02aa1n02x5               g160(.a(new_n253), .b(new_n255), .out0(\s[26] ));
  nano22aa1n02x4               g161(.a(new_n159), .b(new_n165), .c(new_n168), .out0(new_n257));
  aoai13aa1n02x5               g162(.a(new_n257), .b(new_n147), .c(new_n134), .d(new_n144), .o1(new_n258));
  nona23aa1n02x4               g163(.a(new_n258), .b(new_n178), .c(new_n166), .d(new_n179), .out0(new_n259));
  160nm_ficinv00aa1n08x5       g164(.clk(\a[25] ), .clkout(new_n260));
  160nm_ficinv00aa1n08x5       g165(.clk(\a[26] ), .clkout(new_n261));
  xroi22aa1d04x5               g166(.a(new_n260), .b(\b[24] ), .c(new_n261), .d(\b[25] ), .out0(new_n262));
  nano32aa1n02x4               g167(.a(new_n206), .b(new_n262), .c(new_n221), .d(new_n236), .out0(new_n263));
  aoai13aa1n02x5               g168(.a(new_n263), .b(new_n259), .c(new_n133), .d(new_n175), .o1(new_n264));
  oao003aa1n02x5               g169(.a(\a[26] ), .b(\b[25] ), .c(new_n254), .carry(new_n265));
  aobi12aa1n02x5               g170(.a(new_n265), .b(new_n246), .c(new_n262), .out0(new_n266));
  norp02aa1n02x5               g171(.a(\b[26] ), .b(\a[27] ), .o1(new_n267));
  nanp02aa1n02x5               g172(.a(\b[26] ), .b(\a[27] ), .o1(new_n268));
  norb02aa1n02x5               g173(.a(new_n268), .b(new_n267), .out0(new_n269));
  xnbna2aa1n03x5               g174(.a(new_n269), .b(new_n266), .c(new_n264), .out0(\s[27] ));
  160nm_ficinv00aa1n08x5       g175(.clk(new_n267), .clkout(new_n271));
  xnrc02aa1n02x5               g176(.a(\b[27] ), .b(\a[28] ), .out0(new_n272));
  aobi12aa1n02x5               g177(.a(new_n263), .b(new_n182), .c(new_n176), .out0(new_n273));
  aoai13aa1n02x5               g178(.a(new_n236), .b(new_n224), .c(new_n209), .d(new_n221), .o1(new_n274));
  160nm_ficinv00aa1n08x5       g179(.clk(new_n262), .clkout(new_n275));
  aoai13aa1n02x5               g180(.a(new_n265), .b(new_n275), .c(new_n274), .d(new_n245), .o1(new_n276));
  oai012aa1n02x5               g181(.a(new_n268), .b(new_n276), .c(new_n273), .o1(new_n277));
  aoi012aa1n02x5               g182(.a(new_n272), .b(new_n277), .c(new_n271), .o1(new_n278));
  aobi12aa1n02x5               g183(.a(new_n268), .b(new_n266), .c(new_n264), .out0(new_n279));
  nano22aa1n02x4               g184(.a(new_n279), .b(new_n271), .c(new_n272), .out0(new_n280));
  norp02aa1n02x5               g185(.a(new_n278), .b(new_n280), .o1(\s[28] ));
  nano22aa1n02x4               g186(.a(new_n272), .b(new_n271), .c(new_n268), .out0(new_n282));
  oai012aa1n02x5               g187(.a(new_n282), .b(new_n276), .c(new_n273), .o1(new_n283));
  oao003aa1n02x5               g188(.a(\a[28] ), .b(\b[27] ), .c(new_n271), .carry(new_n284));
  xnrc02aa1n02x5               g189(.a(\b[28] ), .b(\a[29] ), .out0(new_n285));
  aoi012aa1n02x5               g190(.a(new_n285), .b(new_n283), .c(new_n284), .o1(new_n286));
  aobi12aa1n02x5               g191(.a(new_n282), .b(new_n266), .c(new_n264), .out0(new_n287));
  nano22aa1n02x4               g192(.a(new_n287), .b(new_n284), .c(new_n285), .out0(new_n288));
  norp02aa1n02x5               g193(.a(new_n286), .b(new_n288), .o1(\s[29] ));
  xorb03aa1n02x5               g194(.a(new_n108), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g195(.a(new_n269), .b(new_n285), .c(new_n272), .out0(new_n291));
  oai012aa1n02x5               g196(.a(new_n291), .b(new_n276), .c(new_n273), .o1(new_n292));
  oao003aa1n02x5               g197(.a(\a[29] ), .b(\b[28] ), .c(new_n284), .carry(new_n293));
  xnrc02aa1n02x5               g198(.a(\b[29] ), .b(\a[30] ), .out0(new_n294));
  aoi012aa1n02x5               g199(.a(new_n294), .b(new_n292), .c(new_n293), .o1(new_n295));
  aobi12aa1n02x5               g200(.a(new_n291), .b(new_n266), .c(new_n264), .out0(new_n296));
  nano22aa1n02x4               g201(.a(new_n296), .b(new_n293), .c(new_n294), .out0(new_n297));
  norp02aa1n02x5               g202(.a(new_n295), .b(new_n297), .o1(\s[30] ));
  norb03aa1n02x5               g203(.a(new_n282), .b(new_n294), .c(new_n285), .out0(new_n299));
  aobi12aa1n02x5               g204(.a(new_n299), .b(new_n266), .c(new_n264), .out0(new_n300));
  oao003aa1n02x5               g205(.a(\a[30] ), .b(\b[29] ), .c(new_n293), .carry(new_n301));
  xnrc02aa1n02x5               g206(.a(\b[30] ), .b(\a[31] ), .out0(new_n302));
  nano22aa1n02x4               g207(.a(new_n300), .b(new_n301), .c(new_n302), .out0(new_n303));
  oai012aa1n02x5               g208(.a(new_n299), .b(new_n276), .c(new_n273), .o1(new_n304));
  aoi012aa1n02x5               g209(.a(new_n302), .b(new_n304), .c(new_n301), .o1(new_n305));
  norp02aa1n02x5               g210(.a(new_n305), .b(new_n303), .o1(\s[31] ));
  xnbna2aa1n03x5               g211(.a(new_n109), .b(new_n103), .c(new_n104), .out0(\s[3] ));
  oaoi03aa1n02x5               g212(.a(\a[3] ), .b(\b[2] ), .c(new_n109), .o1(new_n308));
  xorb03aa1n02x5               g213(.a(new_n308), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g214(.a(new_n112), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oai012aa1n02x5               g215(.a(new_n118), .b(new_n112), .c(new_n117), .o1(new_n311));
  xnrb03aa1n02x5               g216(.a(new_n311), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  nona22aa1n02x4               g217(.a(new_n130), .b(new_n119), .c(new_n100), .out0(new_n313));
  xobna2aa1n03x5               g218(.a(new_n114), .b(new_n313), .c(new_n121), .out0(\s[7] ));
  orn002aa1n02x5               g219(.a(\a[7] ), .b(\b[6] ), .o(new_n315));
  aoai13aa1n02x5               g220(.a(new_n315), .b(new_n114), .c(new_n313), .d(new_n121), .o1(new_n316));
  xorb03aa1n02x5               g221(.a(new_n316), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xorb03aa1n02x5               g222(.a(new_n133), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


