// Benchmark "adder" written by ABC on Thu Jul 11 11:42:40 2024

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
    new_n118, new_n119, new_n120, new_n121, new_n122, new_n123, new_n125,
    new_n126, new_n127, new_n128, new_n129, new_n130, new_n131, new_n132,
    new_n133, new_n134, new_n135, new_n137, new_n138, new_n139, new_n140,
    new_n141, new_n142, new_n143, new_n144, new_n145, new_n146, new_n147,
    new_n148, new_n149, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n158, new_n160, new_n161, new_n162, new_n163, new_n164,
    new_n165, new_n166, new_n167, new_n168, new_n169, new_n170, new_n171,
    new_n172, new_n173, new_n174, new_n176, new_n177, new_n178, new_n179,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n189, new_n190, new_n191, new_n192, new_n194, new_n195, new_n196,
    new_n197, new_n198, new_n199, new_n200, new_n201, new_n202, new_n203,
    new_n204, new_n205, new_n208, new_n209, new_n210, new_n211, new_n212,
    new_n213, new_n214, new_n215, new_n216, new_n217, new_n219, new_n220,
    new_n221, new_n222, new_n223, new_n224, new_n225, new_n226, new_n227,
    new_n228, new_n230, new_n231, new_n232, new_n233, new_n234, new_n235,
    new_n237, new_n238, new_n239, new_n240, new_n241, new_n242, new_n243,
    new_n244, new_n246, new_n247, new_n248, new_n249, new_n250, new_n251,
    new_n252, new_n254, new_n255, new_n256, new_n257, new_n258, new_n259,
    new_n260, new_n261, new_n262, new_n263, new_n264, new_n265, new_n266,
    new_n268, new_n269, new_n270, new_n271, new_n272, new_n273, new_n275,
    new_n276, new_n277, new_n278, new_n279, new_n280, new_n281, new_n282,
    new_n284, new_n285, new_n286, new_n287, new_n288, new_n289, new_n290,
    new_n291, new_n292, new_n293, new_n295, new_n296, new_n297, new_n298,
    new_n299, new_n300, new_n301, new_n304, new_n305, new_n306, new_n307,
    new_n308, new_n309, new_n310, new_n312, new_n313, new_n314, new_n315,
    new_n316, new_n317, new_n318, new_n321, new_n324, new_n326, new_n328;
  160nm_ficinv00aa1n08x5       g000(.clk(\a[0] ), .clkout(\s[0] ));
  160nm_ficinv00aa1n08x5       g001(.clk(\a[2] ), .clkout(new_n97));
  160nm_ficinv00aa1n08x5       g002(.clk(\b[1] ), .clkout(new_n98));
  nanp02aa1n02x5               g003(.a(\b[0] ), .b(\a[1] ), .o1(new_n99));
  oaoi03aa1n02x5               g004(.a(new_n97), .b(new_n98), .c(new_n99), .o1(new_n100));
  norp02aa1n02x5               g005(.a(\b[3] ), .b(\a[4] ), .o1(new_n101));
  nanp02aa1n02x5               g006(.a(\b[3] ), .b(\a[4] ), .o1(new_n102));
  norp02aa1n02x5               g007(.a(\b[2] ), .b(\a[3] ), .o1(new_n103));
  nanp02aa1n02x5               g008(.a(\b[2] ), .b(\a[3] ), .o1(new_n104));
  nona23aa1n02x4               g009(.a(new_n104), .b(new_n102), .c(new_n101), .d(new_n103), .out0(new_n105));
  oai012aa1n02x5               g010(.a(new_n102), .b(new_n103), .c(new_n101), .o1(new_n106));
  oai012aa1n02x5               g011(.a(new_n106), .b(new_n105), .c(new_n100), .o1(new_n107));
  norp02aa1n02x5               g012(.a(\b[7] ), .b(\a[8] ), .o1(new_n108));
  nanp02aa1n02x5               g013(.a(\b[7] ), .b(\a[8] ), .o1(new_n109));
  norp02aa1n02x5               g014(.a(\b[6] ), .b(\a[7] ), .o1(new_n110));
  nanp02aa1n02x5               g015(.a(\b[6] ), .b(\a[7] ), .o1(new_n111));
  nona23aa1n02x4               g016(.a(new_n111), .b(new_n109), .c(new_n108), .d(new_n110), .out0(new_n112));
  xorc02aa1n02x5               g017(.a(\a[6] ), .b(\b[5] ), .out0(new_n113));
  xnrc02aa1n02x5               g018(.a(\b[4] ), .b(\a[5] ), .out0(new_n114));
  norb03aa1n02x5               g019(.a(new_n113), .b(new_n112), .c(new_n114), .out0(new_n115));
  160nm_ficinv00aa1n08x5       g020(.clk(\a[5] ), .clkout(new_n116));
  160nm_ficinv00aa1n08x5       g021(.clk(\b[4] ), .clkout(new_n117));
  nanp02aa1n02x5               g022(.a(new_n117), .b(new_n116), .o1(new_n118));
  oaoi03aa1n02x5               g023(.a(\a[6] ), .b(\b[5] ), .c(new_n118), .o1(new_n119));
  oai012aa1n02x5               g024(.a(new_n109), .b(new_n110), .c(new_n108), .o1(new_n120));
  oaib12aa1n02x5               g025(.a(new_n120), .b(new_n112), .c(new_n119), .out0(new_n121));
  aoi012aa1n02x5               g026(.a(new_n121), .b(new_n115), .c(new_n107), .o1(new_n122));
  oaoi03aa1n02x5               g027(.a(\a[9] ), .b(\b[8] ), .c(new_n122), .o1(new_n123));
  xorb03aa1n02x5               g028(.a(new_n123), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  xnrc02aa1n02x5               g029(.a(\b[9] ), .b(\a[10] ), .out0(new_n125));
  xnrc02aa1n02x5               g030(.a(\b[8] ), .b(\a[9] ), .out0(new_n126));
  norp02aa1n02x5               g031(.a(new_n126), .b(new_n125), .o1(new_n127));
  aoai13aa1n02x5               g032(.a(new_n127), .b(new_n121), .c(new_n115), .d(new_n107), .o1(new_n128));
  160nm_ficinv00aa1n08x5       g033(.clk(\a[10] ), .clkout(new_n129));
  160nm_ficinv00aa1n08x5       g034(.clk(\b[9] ), .clkout(new_n130));
  norp02aa1n02x5               g035(.a(\b[8] ), .b(\a[9] ), .o1(new_n131));
  oaoi03aa1n02x5               g036(.a(new_n129), .b(new_n130), .c(new_n131), .o1(new_n132));
  norp02aa1n02x5               g037(.a(\b[10] ), .b(\a[11] ), .o1(new_n133));
  nanp02aa1n02x5               g038(.a(\b[10] ), .b(\a[11] ), .o1(new_n134));
  norb02aa1n02x5               g039(.a(new_n134), .b(new_n133), .out0(new_n135));
  xnbna2aa1n03x5               g040(.a(new_n135), .b(new_n128), .c(new_n132), .out0(\s[11] ));
  160nm_ficinv00aa1n08x5       g041(.clk(new_n133), .clkout(new_n137));
  oao003aa1n02x5               g042(.a(new_n97), .b(new_n98), .c(new_n99), .carry(new_n138));
  nano23aa1n02x4               g043(.a(new_n101), .b(new_n103), .c(new_n104), .d(new_n102), .out0(new_n139));
  aobi12aa1n02x5               g044(.a(new_n106), .b(new_n139), .c(new_n138), .out0(new_n140));
  nano23aa1n02x4               g045(.a(new_n108), .b(new_n110), .c(new_n111), .d(new_n109), .out0(new_n141));
  nanb03aa1n02x5               g046(.a(new_n114), .b(new_n141), .c(new_n113), .out0(new_n142));
  aobi12aa1n02x5               g047(.a(new_n120), .b(new_n141), .c(new_n119), .out0(new_n143));
  oai012aa1n02x5               g048(.a(new_n143), .b(new_n140), .c(new_n142), .o1(new_n144));
  oao003aa1n02x5               g049(.a(new_n129), .b(new_n130), .c(new_n131), .carry(new_n145));
  aoai13aa1n02x5               g050(.a(new_n135), .b(new_n145), .c(new_n144), .d(new_n127), .o1(new_n146));
  norp02aa1n02x5               g051(.a(\b[11] ), .b(\a[12] ), .o1(new_n147));
  nanp02aa1n02x5               g052(.a(\b[11] ), .b(\a[12] ), .o1(new_n148));
  norb02aa1n02x5               g053(.a(new_n148), .b(new_n147), .out0(new_n149));
  xnbna2aa1n03x5               g054(.a(new_n149), .b(new_n146), .c(new_n137), .out0(\s[12] ));
  160nm_ficinv00aa1n08x5       g055(.clk(\a[13] ), .clkout(new_n151));
  nano23aa1n02x4               g056(.a(new_n133), .b(new_n147), .c(new_n148), .d(new_n134), .out0(new_n152));
  nona23aa1n02x4               g057(.a(new_n148), .b(new_n134), .c(new_n133), .d(new_n147), .out0(new_n153));
  oaoi03aa1n02x5               g058(.a(\a[12] ), .b(\b[11] ), .c(new_n137), .o1(new_n154));
  oabi12aa1n02x5               g059(.a(new_n154), .b(new_n153), .c(new_n132), .out0(new_n155));
  aoi013aa1n02x4               g060(.a(new_n155), .b(new_n144), .c(new_n127), .d(new_n152), .o1(new_n156));
  xorb03aa1n02x5               g061(.a(new_n156), .b(\b[12] ), .c(new_n151), .out0(\s[13] ));
  oaoi03aa1n02x5               g062(.a(\a[13] ), .b(\b[12] ), .c(new_n156), .o1(new_n158));
  xorb03aa1n02x5               g063(.a(new_n158), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  norp02aa1n02x5               g064(.a(\b[12] ), .b(\a[13] ), .o1(new_n160));
  nanp02aa1n02x5               g065(.a(\b[12] ), .b(\a[13] ), .o1(new_n161));
  norp02aa1n02x5               g066(.a(\b[13] ), .b(\a[14] ), .o1(new_n162));
  nanp02aa1n02x5               g067(.a(\b[13] ), .b(\a[14] ), .o1(new_n163));
  nano23aa1n02x4               g068(.a(new_n160), .b(new_n162), .c(new_n163), .d(new_n161), .out0(new_n164));
  160nm_ficinv00aa1n08x5       g069(.clk(\b[12] ), .clkout(new_n165));
  aoai13aa1n02x5               g070(.a(new_n163), .b(new_n162), .c(new_n151), .d(new_n165), .o1(new_n166));
  aobi12aa1n02x5               g071(.a(new_n166), .b(new_n155), .c(new_n164), .out0(new_n167));
  160nm_ficinv00aa1n08x5       g072(.clk(new_n167), .clkout(new_n168));
  nano32aa1n02x4               g073(.a(new_n122), .b(new_n164), .c(new_n127), .d(new_n152), .out0(new_n169));
  norp02aa1n02x5               g074(.a(\b[14] ), .b(\a[15] ), .o1(new_n170));
  nanp02aa1n02x5               g075(.a(\b[14] ), .b(\a[15] ), .o1(new_n171));
  nanb02aa1n02x5               g076(.a(new_n170), .b(new_n171), .out0(new_n172));
  oabi12aa1n02x5               g077(.a(new_n172), .b(new_n169), .c(new_n168), .out0(new_n173));
  nano22aa1n02x4               g078(.a(new_n169), .b(new_n167), .c(new_n172), .out0(new_n174));
  norb02aa1n02x5               g079(.a(new_n173), .b(new_n174), .out0(\s[15] ));
  160nm_ficinv00aa1n08x5       g080(.clk(new_n170), .clkout(new_n176));
  norp02aa1n02x5               g081(.a(\b[15] ), .b(\a[16] ), .o1(new_n177));
  nanp02aa1n02x5               g082(.a(\b[15] ), .b(\a[16] ), .o1(new_n178));
  norb02aa1n02x5               g083(.a(new_n178), .b(new_n177), .out0(new_n179));
  xnbna2aa1n03x5               g084(.a(new_n179), .b(new_n173), .c(new_n176), .out0(\s[16] ));
  nona23aa1n02x4               g085(.a(new_n178), .b(new_n171), .c(new_n170), .d(new_n177), .out0(new_n181));
  160nm_ficinv00aa1n08x5       g086(.clk(new_n127), .clkout(new_n182));
  nano23aa1n02x4               g087(.a(new_n170), .b(new_n177), .c(new_n178), .d(new_n171), .out0(new_n183));
  nano32aa1n02x4               g088(.a(new_n182), .b(new_n183), .c(new_n152), .d(new_n164), .out0(new_n184));
  aoai13aa1n02x5               g089(.a(new_n184), .b(new_n121), .c(new_n107), .d(new_n115), .o1(new_n185));
  oai012aa1n02x5               g090(.a(new_n178), .b(new_n177), .c(new_n170), .o1(new_n186));
  oai112aa1n02x5               g091(.a(new_n185), .b(new_n186), .c(new_n167), .d(new_n181), .o1(new_n187));
  xorb03aa1n02x5               g092(.a(new_n187), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  160nm_ficinv00aa1n08x5       g093(.clk(\a[18] ), .clkout(new_n189));
  160nm_ficinv00aa1n08x5       g094(.clk(\a[17] ), .clkout(new_n190));
  160nm_ficinv00aa1n08x5       g095(.clk(\b[16] ), .clkout(new_n191));
  oaoi03aa1n02x5               g096(.a(new_n190), .b(new_n191), .c(new_n187), .o1(new_n192));
  xorb03aa1n02x5               g097(.a(new_n192), .b(\b[17] ), .c(new_n189), .out0(\s[18] ));
  nona23aa1n02x4               g098(.a(new_n164), .b(new_n127), .c(new_n181), .d(new_n153), .out0(new_n194));
  oaoi13aa1n02x5               g099(.a(new_n194), .b(new_n143), .c(new_n140), .d(new_n142), .o1(new_n195));
  aoai13aa1n02x5               g100(.a(new_n164), .b(new_n154), .c(new_n152), .d(new_n145), .o1(new_n196));
  aoai13aa1n02x5               g101(.a(new_n186), .b(new_n181), .c(new_n196), .d(new_n166), .o1(new_n197));
  xroi22aa1d04x5               g102(.a(new_n190), .b(\b[16] ), .c(new_n189), .d(\b[17] ), .out0(new_n198));
  oai012aa1n02x5               g103(.a(new_n198), .b(new_n197), .c(new_n195), .o1(new_n199));
  oai022aa1n02x5               g104(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n200));
  oaib12aa1n02x5               g105(.a(new_n200), .b(new_n189), .c(\b[17] ), .out0(new_n201));
  norp02aa1n02x5               g106(.a(\b[18] ), .b(\a[19] ), .o1(new_n202));
  nanp02aa1n02x5               g107(.a(\b[18] ), .b(\a[19] ), .o1(new_n203));
  nanb02aa1n02x5               g108(.a(new_n202), .b(new_n203), .out0(new_n204));
  160nm_ficinv00aa1n08x5       g109(.clk(new_n204), .clkout(new_n205));
  xnbna2aa1n03x5               g110(.a(new_n205), .b(new_n199), .c(new_n201), .out0(\s[19] ));
  xnrc02aa1n02x5               g111(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  160nm_ficinv00aa1n08x5       g112(.clk(new_n202), .clkout(new_n208));
  aoi012aa1n02x5               g113(.a(new_n204), .b(new_n199), .c(new_n201), .o1(new_n209));
  norp02aa1n02x5               g114(.a(\b[19] ), .b(\a[20] ), .o1(new_n210));
  nanp02aa1n02x5               g115(.a(\b[19] ), .b(\a[20] ), .o1(new_n211));
  nanb02aa1n02x5               g116(.a(new_n210), .b(new_n211), .out0(new_n212));
  nano22aa1n02x4               g117(.a(new_n209), .b(new_n208), .c(new_n212), .out0(new_n213));
  nanp02aa1n02x5               g118(.a(new_n191), .b(new_n190), .o1(new_n214));
  oaoi03aa1n02x5               g119(.a(\a[18] ), .b(\b[17] ), .c(new_n214), .o1(new_n215));
  aoai13aa1n02x5               g120(.a(new_n205), .b(new_n215), .c(new_n187), .d(new_n198), .o1(new_n216));
  aoi012aa1n02x5               g121(.a(new_n212), .b(new_n216), .c(new_n208), .o1(new_n217));
  norp02aa1n02x5               g122(.a(new_n217), .b(new_n213), .o1(\s[20] ));
  nano23aa1n02x4               g123(.a(new_n202), .b(new_n210), .c(new_n211), .d(new_n203), .out0(new_n219));
  nanp02aa1n02x5               g124(.a(new_n198), .b(new_n219), .o1(new_n220));
  160nm_ficinv00aa1n08x5       g125(.clk(new_n220), .clkout(new_n221));
  oai012aa1n02x5               g126(.a(new_n221), .b(new_n197), .c(new_n195), .o1(new_n222));
  nona23aa1n02x4               g127(.a(new_n211), .b(new_n203), .c(new_n202), .d(new_n210), .out0(new_n223));
  oaoi03aa1n02x5               g128(.a(\a[20] ), .b(\b[19] ), .c(new_n208), .o1(new_n224));
  160nm_ficinv00aa1n08x5       g129(.clk(new_n224), .clkout(new_n225));
  oai012aa1n02x5               g130(.a(new_n225), .b(new_n223), .c(new_n201), .o1(new_n226));
  160nm_ficinv00aa1n08x5       g131(.clk(new_n226), .clkout(new_n227));
  xorc02aa1n02x5               g132(.a(\a[21] ), .b(\b[20] ), .out0(new_n228));
  xnbna2aa1n03x5               g133(.a(new_n228), .b(new_n222), .c(new_n227), .out0(\s[21] ));
  orn002aa1n02x5               g134(.a(\a[21] ), .b(\b[20] ), .o(new_n230));
  aobi12aa1n02x5               g135(.a(new_n228), .b(new_n222), .c(new_n227), .out0(new_n231));
  xnrc02aa1n02x5               g136(.a(\b[21] ), .b(\a[22] ), .out0(new_n232));
  nano22aa1n02x4               g137(.a(new_n231), .b(new_n230), .c(new_n232), .out0(new_n233));
  aoai13aa1n02x5               g138(.a(new_n228), .b(new_n226), .c(new_n187), .d(new_n221), .o1(new_n234));
  aoi012aa1n02x5               g139(.a(new_n232), .b(new_n234), .c(new_n230), .o1(new_n235));
  norp02aa1n02x5               g140(.a(new_n235), .b(new_n233), .o1(\s[22] ));
  nanp02aa1n02x5               g141(.a(\b[20] ), .b(\a[21] ), .o1(new_n237));
  nano22aa1n02x4               g142(.a(new_n232), .b(new_n230), .c(new_n237), .out0(new_n238));
  and003aa1n02x5               g143(.a(new_n198), .b(new_n238), .c(new_n219), .o(new_n239));
  oai012aa1n02x5               g144(.a(new_n239), .b(new_n197), .c(new_n195), .o1(new_n240));
  oaoi03aa1n02x5               g145(.a(\a[22] ), .b(\b[21] ), .c(new_n230), .o1(new_n241));
  aoi012aa1n02x5               g146(.a(new_n241), .b(new_n226), .c(new_n238), .o1(new_n242));
  xnrc02aa1n02x5               g147(.a(\b[22] ), .b(\a[23] ), .out0(new_n243));
  160nm_ficinv00aa1n08x5       g148(.clk(new_n243), .clkout(new_n244));
  xnbna2aa1n03x5               g149(.a(new_n244), .b(new_n240), .c(new_n242), .out0(\s[23] ));
  orn002aa1n02x5               g150(.a(\a[23] ), .b(\b[22] ), .o(new_n246));
  aoi012aa1n02x5               g151(.a(new_n243), .b(new_n240), .c(new_n242), .o1(new_n247));
  xnrc02aa1n02x5               g152(.a(\b[23] ), .b(\a[24] ), .out0(new_n248));
  nano22aa1n02x4               g153(.a(new_n247), .b(new_n246), .c(new_n248), .out0(new_n249));
  160nm_ficinv00aa1n08x5       g154(.clk(new_n242), .clkout(new_n250));
  aoai13aa1n02x5               g155(.a(new_n244), .b(new_n250), .c(new_n187), .d(new_n239), .o1(new_n251));
  aoi012aa1n02x5               g156(.a(new_n248), .b(new_n251), .c(new_n246), .o1(new_n252));
  norp02aa1n02x5               g157(.a(new_n252), .b(new_n249), .o1(\s[24] ));
  norp02aa1n02x5               g158(.a(new_n248), .b(new_n243), .o1(new_n254));
  nano22aa1n02x4               g159(.a(new_n220), .b(new_n238), .c(new_n254), .out0(new_n255));
  oai012aa1n02x5               g160(.a(new_n255), .b(new_n197), .c(new_n195), .o1(new_n256));
  aoai13aa1n02x5               g161(.a(new_n238), .b(new_n224), .c(new_n219), .d(new_n215), .o1(new_n257));
  160nm_ficinv00aa1n08x5       g162(.clk(new_n241), .clkout(new_n258));
  160nm_ficinv00aa1n08x5       g163(.clk(new_n254), .clkout(new_n259));
  oaoi03aa1n02x5               g164(.a(\a[24] ), .b(\b[23] ), .c(new_n246), .o1(new_n260));
  160nm_ficinv00aa1n08x5       g165(.clk(new_n260), .clkout(new_n261));
  aoai13aa1n02x5               g166(.a(new_n261), .b(new_n259), .c(new_n257), .d(new_n258), .o1(new_n262));
  xnrc02aa1n02x5               g167(.a(\b[24] ), .b(\a[25] ), .out0(new_n263));
  aoib12aa1n02x5               g168(.a(new_n263), .b(new_n256), .c(new_n262), .out0(new_n264));
  160nm_ficinv00aa1n08x5       g169(.clk(new_n263), .clkout(new_n265));
  aoi112aa1n02x5               g170(.a(new_n265), .b(new_n262), .c(new_n187), .d(new_n255), .o1(new_n266));
  norp02aa1n02x5               g171(.a(new_n264), .b(new_n266), .o1(\s[25] ));
  norp02aa1n02x5               g172(.a(\b[24] ), .b(\a[25] ), .o1(new_n268));
  160nm_ficinv00aa1n08x5       g173(.clk(new_n268), .clkout(new_n269));
  xnrc02aa1n02x5               g174(.a(\b[25] ), .b(\a[26] ), .out0(new_n270));
  nano22aa1n02x4               g175(.a(new_n264), .b(new_n269), .c(new_n270), .out0(new_n271));
  aoai13aa1n02x5               g176(.a(new_n265), .b(new_n262), .c(new_n187), .d(new_n255), .o1(new_n272));
  aoi012aa1n02x5               g177(.a(new_n270), .b(new_n272), .c(new_n269), .o1(new_n273));
  norp02aa1n02x5               g178(.a(new_n273), .b(new_n271), .o1(\s[26] ));
  norp02aa1n02x5               g179(.a(new_n270), .b(new_n263), .o1(new_n275));
  nano32aa1n02x4               g180(.a(new_n220), .b(new_n275), .c(new_n238), .d(new_n254), .out0(new_n276));
  oai012aa1n02x5               g181(.a(new_n276), .b(new_n197), .c(new_n195), .o1(new_n277));
  oao003aa1n02x5               g182(.a(\a[26] ), .b(\b[25] ), .c(new_n269), .carry(new_n278));
  aobi12aa1n02x5               g183(.a(new_n278), .b(new_n262), .c(new_n275), .out0(new_n279));
  norp02aa1n02x5               g184(.a(\b[26] ), .b(\a[27] ), .o1(new_n280));
  nanp02aa1n02x5               g185(.a(\b[26] ), .b(\a[27] ), .o1(new_n281));
  norb02aa1n02x5               g186(.a(new_n281), .b(new_n280), .out0(new_n282));
  xnbna2aa1n03x5               g187(.a(new_n282), .b(new_n277), .c(new_n279), .out0(\s[27] ));
  norp02aa1n02x5               g188(.a(\b[27] ), .b(\a[28] ), .o1(new_n284));
  nanp02aa1n02x5               g189(.a(\b[27] ), .b(\a[28] ), .o1(new_n285));
  norb02aa1n02x5               g190(.a(new_n285), .b(new_n284), .out0(new_n286));
  aoai13aa1n02x5               g191(.a(new_n254), .b(new_n241), .c(new_n226), .d(new_n238), .o1(new_n287));
  160nm_ficinv00aa1n08x5       g192(.clk(new_n275), .clkout(new_n288));
  aoai13aa1n02x5               g193(.a(new_n278), .b(new_n288), .c(new_n287), .d(new_n261), .o1(new_n289));
  aoi112aa1n02x5               g194(.a(new_n289), .b(new_n280), .c(new_n187), .d(new_n276), .o1(new_n290));
  nano22aa1n02x4               g195(.a(new_n290), .b(new_n281), .c(new_n286), .out0(new_n291));
  oai112aa1n02x5               g196(.a(new_n277), .b(new_n279), .c(\b[26] ), .d(\a[27] ), .o1(new_n292));
  aoi012aa1n02x5               g197(.a(new_n286), .b(new_n292), .c(new_n281), .o1(new_n293));
  norp02aa1n02x5               g198(.a(new_n293), .b(new_n291), .o1(\s[28] ));
  nano23aa1n02x4               g199(.a(new_n280), .b(new_n284), .c(new_n285), .d(new_n281), .out0(new_n295));
  aoai13aa1n02x5               g200(.a(new_n295), .b(new_n289), .c(new_n187), .d(new_n276), .o1(new_n296));
  oai012aa1n02x5               g201(.a(new_n285), .b(new_n284), .c(new_n280), .o1(new_n297));
  xnrc02aa1n02x5               g202(.a(\b[28] ), .b(\a[29] ), .out0(new_n298));
  aoi012aa1n02x5               g203(.a(new_n298), .b(new_n296), .c(new_n297), .o1(new_n299));
  aobi12aa1n02x5               g204(.a(new_n295), .b(new_n277), .c(new_n279), .out0(new_n300));
  nano22aa1n02x4               g205(.a(new_n300), .b(new_n297), .c(new_n298), .out0(new_n301));
  norp02aa1n02x5               g206(.a(new_n299), .b(new_n301), .o1(\s[29] ));
  xorb03aa1n02x5               g207(.a(new_n99), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n02x4               g208(.a(new_n298), .b(new_n282), .c(new_n286), .out0(new_n304));
  aoai13aa1n02x5               g209(.a(new_n304), .b(new_n289), .c(new_n187), .d(new_n276), .o1(new_n305));
  oao003aa1n02x5               g210(.a(\a[29] ), .b(\b[28] ), .c(new_n297), .carry(new_n306));
  xnrc02aa1n02x5               g211(.a(\b[29] ), .b(\a[30] ), .out0(new_n307));
  aoi012aa1n02x5               g212(.a(new_n307), .b(new_n305), .c(new_n306), .o1(new_n308));
  aobi12aa1n02x5               g213(.a(new_n304), .b(new_n277), .c(new_n279), .out0(new_n309));
  nano22aa1n02x4               g214(.a(new_n309), .b(new_n306), .c(new_n307), .out0(new_n310));
  norp02aa1n02x5               g215(.a(new_n308), .b(new_n310), .o1(\s[30] ));
  nano23aa1n02x4               g216(.a(new_n307), .b(new_n298), .c(new_n286), .d(new_n282), .out0(new_n312));
  aobi12aa1n02x5               g217(.a(new_n312), .b(new_n277), .c(new_n279), .out0(new_n313));
  oao003aa1n02x5               g218(.a(\a[30] ), .b(\b[29] ), .c(new_n306), .carry(new_n314));
  xnrc02aa1n02x5               g219(.a(\b[30] ), .b(\a[31] ), .out0(new_n315));
  nano22aa1n02x4               g220(.a(new_n313), .b(new_n314), .c(new_n315), .out0(new_n316));
  aoai13aa1n02x5               g221(.a(new_n312), .b(new_n289), .c(new_n187), .d(new_n276), .o1(new_n317));
  aoi012aa1n02x5               g222(.a(new_n315), .b(new_n317), .c(new_n314), .o1(new_n318));
  norp02aa1n02x5               g223(.a(new_n318), .b(new_n316), .o1(\s[31] ));
  xnrb03aa1n02x5               g224(.a(new_n100), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g225(.a(\a[3] ), .b(\b[2] ), .c(new_n100), .o1(new_n321));
  xorb03aa1n02x5               g226(.a(new_n321), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g227(.a(new_n107), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oaoi03aa1n02x5               g228(.a(new_n116), .b(new_n117), .c(new_n107), .o1(new_n324));
  xnrc02aa1n02x5               g229(.a(new_n324), .b(new_n113), .out0(\s[6] ));
  oaoi03aa1n02x5               g230(.a(\a[6] ), .b(\b[5] ), .c(new_n324), .o1(new_n326));
  xorb03aa1n02x5               g231(.a(new_n326), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoi012aa1n02x5               g232(.a(new_n110), .b(new_n326), .c(new_n111), .o1(new_n328));
  xnrb03aa1n02x5               g233(.a(new_n328), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xorb03aa1n02x5               g234(.a(new_n144), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


