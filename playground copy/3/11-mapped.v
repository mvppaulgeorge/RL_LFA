// Benchmark "adder" written by ABC on Wed Jul 10 17:27:17 2024

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
    new_n133, new_n134, new_n136, new_n137, new_n138, new_n139, new_n140,
    new_n141, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n154, new_n155, new_n156,
    new_n158, new_n159, new_n160, new_n161, new_n162, new_n163, new_n164,
    new_n166, new_n167, new_n168, new_n169, new_n170, new_n171, new_n172,
    new_n173, new_n175, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n189, new_n190, new_n191, new_n192, new_n194, new_n195, new_n196,
    new_n197, new_n198, new_n201, new_n202, new_n203, new_n204, new_n205,
    new_n206, new_n207, new_n208, new_n209, new_n211, new_n212, new_n213,
    new_n214, new_n215, new_n217, new_n218, new_n219, new_n220, new_n221,
    new_n222, new_n223, new_n224, new_n226, new_n227, new_n228, new_n229,
    new_n230, new_n231, new_n232, new_n234, new_n235, new_n236, new_n237,
    new_n238, new_n239, new_n240, new_n241, new_n243, new_n244, new_n245,
    new_n246, new_n247, new_n248, new_n249, new_n250, new_n251, new_n252,
    new_n253, new_n254, new_n255, new_n257, new_n258, new_n259, new_n260,
    new_n261, new_n262, new_n263, new_n264, new_n266, new_n267, new_n268,
    new_n269, new_n270, new_n271, new_n272, new_n273, new_n274, new_n275,
    new_n276, new_n277, new_n278, new_n280, new_n281, new_n282, new_n283,
    new_n284, new_n285, new_n286, new_n287, new_n288, new_n289, new_n291,
    new_n292, new_n293, new_n294, new_n295, new_n296, new_n297, new_n298,
    new_n301, new_n302, new_n303, new_n304, new_n305, new_n306, new_n307,
    new_n308, new_n310, new_n311, new_n312, new_n313, new_n314, new_n315,
    new_n316, new_n317, new_n320, new_n323, new_n324, new_n326, new_n328;
  160nm_ficinv00aa1n08x5       g000(.clk(\a[0] ), .clkout(\s[0] ));
  norp02aa1n02x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  160nm_ficinv00aa1n08x5       g002(.clk(new_n97), .clkout(new_n98));
  norp02aa1n02x5               g003(.a(\b[7] ), .b(\a[8] ), .o1(new_n99));
  nanp02aa1n02x5               g004(.a(\b[7] ), .b(\a[8] ), .o1(new_n100));
  norp02aa1n02x5               g005(.a(\b[6] ), .b(\a[7] ), .o1(new_n101));
  nanp02aa1n02x5               g006(.a(\b[6] ), .b(\a[7] ), .o1(new_n102));
  nona23aa1n02x4               g007(.a(new_n102), .b(new_n100), .c(new_n99), .d(new_n101), .out0(new_n103));
  xnrc02aa1n02x5               g008(.a(\b[5] ), .b(\a[6] ), .out0(new_n104));
  xnrc02aa1n02x5               g009(.a(\b[4] ), .b(\a[5] ), .out0(new_n105));
  norp03aa1n02x5               g010(.a(new_n103), .b(new_n104), .c(new_n105), .o1(new_n106));
  nanp02aa1n02x5               g011(.a(\b[1] ), .b(\a[2] ), .o1(new_n107));
  nanp02aa1n02x5               g012(.a(\b[0] ), .b(\a[1] ), .o1(new_n108));
  norp02aa1n02x5               g013(.a(\b[1] ), .b(\a[2] ), .o1(new_n109));
  oai012aa1n02x5               g014(.a(new_n107), .b(new_n109), .c(new_n108), .o1(new_n110));
  norp02aa1n02x5               g015(.a(\b[3] ), .b(\a[4] ), .o1(new_n111));
  nanp02aa1n02x5               g016(.a(\b[3] ), .b(\a[4] ), .o1(new_n112));
  norp02aa1n02x5               g017(.a(\b[2] ), .b(\a[3] ), .o1(new_n113));
  nanp02aa1n02x5               g018(.a(\b[2] ), .b(\a[3] ), .o1(new_n114));
  nona23aa1n02x4               g019(.a(new_n114), .b(new_n112), .c(new_n111), .d(new_n113), .out0(new_n115));
  aoi012aa1n02x5               g020(.a(new_n111), .b(new_n113), .c(new_n112), .o1(new_n116));
  oai012aa1n02x5               g021(.a(new_n116), .b(new_n115), .c(new_n110), .o1(new_n117));
  oai022aa1n02x5               g022(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n118));
  aob012aa1n02x5               g023(.a(new_n118), .b(\b[5] ), .c(\a[6] ), .out0(new_n119));
  160nm_fiao0012aa1n02p5x5     g024(.a(new_n99), .b(new_n101), .c(new_n100), .o(new_n120));
  oabi12aa1n02x5               g025(.a(new_n120), .b(new_n103), .c(new_n119), .out0(new_n121));
  nanp02aa1n02x5               g026(.a(\b[8] ), .b(\a[9] ), .o1(new_n122));
  norb02aa1n02x5               g027(.a(new_n122), .b(new_n97), .out0(new_n123));
  aoai13aa1n02x5               g028(.a(new_n123), .b(new_n121), .c(new_n117), .d(new_n106), .o1(new_n124));
  norp02aa1n02x5               g029(.a(\b[9] ), .b(\a[10] ), .o1(new_n125));
  nanp02aa1n02x5               g030(.a(\b[9] ), .b(\a[10] ), .o1(new_n126));
  norb02aa1n02x5               g031(.a(new_n126), .b(new_n125), .out0(new_n127));
  xnbna2aa1n03x5               g032(.a(new_n127), .b(new_n124), .c(new_n98), .out0(\s[10] ));
  norp02aa1n02x5               g033(.a(\b[10] ), .b(\a[11] ), .o1(new_n129));
  nanp02aa1n02x5               g034(.a(\b[10] ), .b(\a[11] ), .o1(new_n130));
  norb02aa1n02x5               g035(.a(new_n130), .b(new_n129), .out0(new_n131));
  oai012aa1n02x5               g036(.a(new_n126), .b(new_n125), .c(new_n97), .o1(new_n132));
  nano23aa1n02x4               g037(.a(new_n97), .b(new_n125), .c(new_n126), .d(new_n122), .out0(new_n133));
  aoai13aa1n02x5               g038(.a(new_n133), .b(new_n121), .c(new_n117), .d(new_n106), .o1(new_n134));
  xnbna2aa1n03x5               g039(.a(new_n131), .b(new_n134), .c(new_n132), .out0(\s[11] ));
  nanp02aa1n02x5               g040(.a(new_n117), .b(new_n106), .o1(new_n136));
  160nm_ficinv00aa1n08x5       g041(.clk(new_n121), .clkout(new_n137));
  nanp02aa1n02x5               g042(.a(new_n136), .b(new_n137), .o1(new_n138));
  oaoi03aa1n02x5               g043(.a(\a[10] ), .b(\b[9] ), .c(new_n98), .o1(new_n139));
  aoai13aa1n02x5               g044(.a(new_n131), .b(new_n139), .c(new_n138), .d(new_n133), .o1(new_n140));
  oai012aa1n02x5               g045(.a(new_n140), .b(\b[10] ), .c(\a[11] ), .o1(new_n141));
  xorb03aa1n02x5               g046(.a(new_n141), .b(\b[11] ), .c(\a[12] ), .out0(\s[12] ));
  norp02aa1n02x5               g047(.a(\b[11] ), .b(\a[12] ), .o1(new_n143));
  nanp02aa1n02x5               g048(.a(\b[11] ), .b(\a[12] ), .o1(new_n144));
  nanb02aa1n02x5               g049(.a(new_n143), .b(new_n144), .out0(new_n145));
  nano32aa1n02x4               g050(.a(new_n145), .b(new_n131), .c(new_n127), .d(new_n123), .out0(new_n146));
  aoai13aa1n02x5               g051(.a(new_n146), .b(new_n121), .c(new_n117), .d(new_n106), .o1(new_n147));
  nano23aa1n02x4               g052(.a(new_n129), .b(new_n143), .c(new_n144), .d(new_n130), .out0(new_n148));
  aoi012aa1n02x5               g053(.a(new_n143), .b(new_n129), .c(new_n144), .o1(new_n149));
  160nm_ficinv00aa1n08x5       g054(.clk(new_n149), .clkout(new_n150));
  aoi012aa1n02x5               g055(.a(new_n150), .b(new_n148), .c(new_n139), .o1(new_n151));
  nanp02aa1n02x5               g056(.a(new_n147), .b(new_n151), .o1(new_n152));
  xorb03aa1n02x5               g057(.a(new_n152), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  160nm_ficinv00aa1n08x5       g058(.clk(\a[13] ), .clkout(new_n154));
  160nm_ficinv00aa1n08x5       g059(.clk(\b[12] ), .clkout(new_n155));
  oaoi03aa1n02x5               g060(.a(new_n154), .b(new_n155), .c(new_n152), .o1(new_n156));
  xnrb03aa1n02x5               g061(.a(new_n156), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  norp02aa1n02x5               g062(.a(\b[12] ), .b(\a[13] ), .o1(new_n158));
  nanp02aa1n02x5               g063(.a(\b[12] ), .b(\a[13] ), .o1(new_n159));
  norp02aa1n02x5               g064(.a(\b[13] ), .b(\a[14] ), .o1(new_n160));
  nanp02aa1n02x5               g065(.a(\b[13] ), .b(\a[14] ), .o1(new_n161));
  nona23aa1n02x4               g066(.a(new_n161), .b(new_n159), .c(new_n158), .d(new_n160), .out0(new_n162));
  aoai13aa1n02x5               g067(.a(new_n161), .b(new_n160), .c(new_n154), .d(new_n155), .o1(new_n163));
  aoai13aa1n02x5               g068(.a(new_n163), .b(new_n162), .c(new_n147), .d(new_n151), .o1(new_n164));
  xorb03aa1n02x5               g069(.a(new_n164), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  norp02aa1n02x5               g070(.a(\b[14] ), .b(\a[15] ), .o1(new_n166));
  nanp02aa1n02x5               g071(.a(\b[14] ), .b(\a[15] ), .o1(new_n167));
  norp02aa1n02x5               g072(.a(\b[15] ), .b(\a[16] ), .o1(new_n168));
  nanp02aa1n02x5               g073(.a(\b[15] ), .b(\a[16] ), .o1(new_n169));
  nanb02aa1n02x5               g074(.a(new_n168), .b(new_n169), .out0(new_n170));
  160nm_ficinv00aa1n08x5       g075(.clk(new_n170), .clkout(new_n171));
  aoi112aa1n02x5               g076(.a(new_n171), .b(new_n166), .c(new_n164), .d(new_n167), .o1(new_n172));
  aoai13aa1n02x5               g077(.a(new_n171), .b(new_n166), .c(new_n164), .d(new_n167), .o1(new_n173));
  norb02aa1n02x5               g078(.a(new_n173), .b(new_n172), .out0(\s[16] ));
  nano23aa1n02x4               g079(.a(new_n158), .b(new_n160), .c(new_n161), .d(new_n159), .out0(new_n175));
  nano23aa1n02x4               g080(.a(new_n166), .b(new_n168), .c(new_n169), .d(new_n167), .out0(new_n176));
  nanp02aa1n02x5               g081(.a(new_n176), .b(new_n175), .o1(new_n177));
  nano22aa1n02x4               g082(.a(new_n177), .b(new_n133), .c(new_n148), .out0(new_n178));
  aoai13aa1n02x5               g083(.a(new_n178), .b(new_n121), .c(new_n117), .d(new_n106), .o1(new_n179));
  160nm_ficinv00aa1n08x5       g084(.clk(new_n168), .clkout(new_n180));
  nanb02aa1n02x5               g085(.a(new_n163), .b(new_n176), .out0(new_n181));
  aoi112aa1n02x5               g086(.a(\b[14] ), .b(\a[15] ), .c(\a[16] ), .d(\b[15] ), .o1(new_n182));
  160nm_ficinv00aa1n08x5       g087(.clk(new_n182), .clkout(new_n183));
  nanp02aa1n02x5               g088(.a(new_n148), .b(new_n139), .o1(new_n184));
  aoi012aa1n02x5               g089(.a(new_n177), .b(new_n184), .c(new_n149), .o1(new_n185));
  nano32aa1n02x4               g090(.a(new_n185), .b(new_n183), .c(new_n181), .d(new_n180), .out0(new_n186));
  xorc02aa1n02x5               g091(.a(\a[17] ), .b(\b[16] ), .out0(new_n187));
  xnbna2aa1n03x5               g092(.a(new_n187), .b(new_n186), .c(new_n179), .out0(\s[17] ));
  nanp02aa1n02x5               g093(.a(new_n186), .b(new_n179), .o1(new_n189));
  norp02aa1n02x5               g094(.a(\b[16] ), .b(\a[17] ), .o1(new_n190));
  aoi012aa1n02x5               g095(.a(new_n190), .b(new_n189), .c(new_n187), .o1(new_n191));
  xnrc02aa1n02x5               g096(.a(\b[17] ), .b(\a[18] ), .out0(new_n192));
  xorc02aa1n02x5               g097(.a(new_n191), .b(new_n192), .out0(\s[18] ));
  norb02aa1n02x5               g098(.a(new_n187), .b(new_n192), .out0(new_n194));
  160nm_ficinv00aa1n08x5       g099(.clk(new_n194), .clkout(new_n195));
  oai022aa1n02x5               g100(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n196));
  aob012aa1n02x5               g101(.a(new_n196), .b(\b[17] ), .c(\a[18] ), .out0(new_n197));
  aoai13aa1n02x5               g102(.a(new_n197), .b(new_n195), .c(new_n186), .d(new_n179), .o1(new_n198));
  xorb03aa1n02x5               g103(.a(new_n198), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g104(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  norp02aa1n02x5               g105(.a(\b[18] ), .b(\a[19] ), .o1(new_n201));
  xnrc02aa1n02x5               g106(.a(\b[18] ), .b(\a[19] ), .out0(new_n202));
  160nm_ficinv00aa1n08x5       g107(.clk(new_n202), .clkout(new_n203));
  xnrc02aa1n02x5               g108(.a(\b[19] ), .b(\a[20] ), .out0(new_n204));
  160nm_ficinv00aa1n08x5       g109(.clk(new_n204), .clkout(new_n205));
  aoi112aa1n02x5               g110(.a(new_n201), .b(new_n205), .c(new_n198), .d(new_n203), .o1(new_n206));
  160nm_ficinv00aa1n08x5       g111(.clk(new_n201), .clkout(new_n207));
  nanp02aa1n02x5               g112(.a(new_n198), .b(new_n203), .o1(new_n208));
  aoi012aa1n02x5               g113(.a(new_n204), .b(new_n208), .c(new_n207), .o1(new_n209));
  norp02aa1n02x5               g114(.a(new_n209), .b(new_n206), .o1(\s[20] ));
  nona23aa1n02x4               g115(.a(new_n187), .b(new_n205), .c(new_n202), .d(new_n192), .out0(new_n211));
  oao003aa1n02x5               g116(.a(\a[20] ), .b(\b[19] ), .c(new_n207), .carry(new_n212));
  oai013aa1n02x4               g117(.a(new_n212), .b(new_n197), .c(new_n202), .d(new_n204), .o1(new_n213));
  160nm_ficinv00aa1n08x5       g118(.clk(new_n213), .clkout(new_n214));
  aoai13aa1n02x5               g119(.a(new_n214), .b(new_n211), .c(new_n186), .d(new_n179), .o1(new_n215));
  xorb03aa1n02x5               g120(.a(new_n215), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g121(.a(\b[20] ), .b(\a[21] ), .o1(new_n217));
  xorc02aa1n02x5               g122(.a(\a[21] ), .b(\b[20] ), .out0(new_n218));
  xorc02aa1n02x5               g123(.a(\a[22] ), .b(\b[21] ), .out0(new_n219));
  aoi112aa1n02x5               g124(.a(new_n217), .b(new_n219), .c(new_n215), .d(new_n218), .o1(new_n220));
  160nm_ficinv00aa1n08x5       g125(.clk(new_n217), .clkout(new_n221));
  nanp02aa1n02x5               g126(.a(new_n215), .b(new_n218), .o1(new_n222));
  160nm_ficinv00aa1n08x5       g127(.clk(new_n219), .clkout(new_n223));
  aoi012aa1n02x5               g128(.a(new_n223), .b(new_n222), .c(new_n221), .o1(new_n224));
  norp02aa1n02x5               g129(.a(new_n224), .b(new_n220), .o1(\s[22] ));
  160nm_ficinv00aa1n08x5       g130(.clk(\a[21] ), .clkout(new_n226));
  160nm_ficinv00aa1n08x5       g131(.clk(\a[22] ), .clkout(new_n227));
  xroi22aa1d04x5               g132(.a(new_n226), .b(\b[20] ), .c(new_n227), .d(\b[21] ), .out0(new_n228));
  nona23aa1n02x4               g133(.a(new_n194), .b(new_n228), .c(new_n204), .d(new_n202), .out0(new_n229));
  oaoi03aa1n02x5               g134(.a(\a[22] ), .b(\b[21] ), .c(new_n221), .o1(new_n230));
  aoi012aa1n02x5               g135(.a(new_n230), .b(new_n213), .c(new_n228), .o1(new_n231));
  aoai13aa1n02x5               g136(.a(new_n231), .b(new_n229), .c(new_n186), .d(new_n179), .o1(new_n232));
  xorb03aa1n02x5               g137(.a(new_n232), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g138(.a(\b[22] ), .b(\a[23] ), .o1(new_n234));
  xorc02aa1n02x5               g139(.a(\a[23] ), .b(\b[22] ), .out0(new_n235));
  xorc02aa1n02x5               g140(.a(\a[24] ), .b(\b[23] ), .out0(new_n236));
  aoi112aa1n02x5               g141(.a(new_n234), .b(new_n236), .c(new_n232), .d(new_n235), .o1(new_n237));
  160nm_ficinv00aa1n08x5       g142(.clk(new_n234), .clkout(new_n238));
  nanp02aa1n02x5               g143(.a(new_n232), .b(new_n235), .o1(new_n239));
  160nm_ficinv00aa1n08x5       g144(.clk(new_n236), .clkout(new_n240));
  aoi012aa1n02x5               g145(.a(new_n240), .b(new_n239), .c(new_n238), .o1(new_n241));
  norp02aa1n02x5               g146(.a(new_n241), .b(new_n237), .o1(\s[24] ));
  nanp02aa1n02x5               g147(.a(new_n219), .b(new_n218), .o1(new_n243));
  nano22aa1n02x4               g148(.a(new_n243), .b(new_n235), .c(new_n236), .out0(new_n244));
  nanb02aa1n02x5               g149(.a(new_n211), .b(new_n244), .out0(new_n245));
  nona22aa1n02x4               g150(.a(new_n205), .b(new_n197), .c(new_n202), .out0(new_n246));
  160nm_ficinv00aa1n08x5       g151(.clk(\a[23] ), .clkout(new_n247));
  160nm_ficinv00aa1n08x5       g152(.clk(\a[24] ), .clkout(new_n248));
  xroi22aa1d04x5               g153(.a(new_n247), .b(\b[22] ), .c(new_n248), .d(\b[23] ), .out0(new_n249));
  nanp02aa1n02x5               g154(.a(new_n249), .b(new_n228), .o1(new_n250));
  oaoi03aa1n02x5               g155(.a(\a[24] ), .b(\b[23] ), .c(new_n238), .o1(new_n251));
  aoi013aa1n02x4               g156(.a(new_n251), .b(new_n230), .c(new_n235), .d(new_n236), .o1(new_n252));
  aoai13aa1n02x5               g157(.a(new_n252), .b(new_n250), .c(new_n246), .d(new_n212), .o1(new_n253));
  160nm_ficinv00aa1n08x5       g158(.clk(new_n253), .clkout(new_n254));
  aoai13aa1n02x5               g159(.a(new_n254), .b(new_n245), .c(new_n186), .d(new_n179), .o1(new_n255));
  xorb03aa1n02x5               g160(.a(new_n255), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g161(.a(\b[24] ), .b(\a[25] ), .o1(new_n257));
  xorc02aa1n02x5               g162(.a(\a[25] ), .b(\b[24] ), .out0(new_n258));
  xorc02aa1n02x5               g163(.a(\a[26] ), .b(\b[25] ), .out0(new_n259));
  aoi112aa1n02x5               g164(.a(new_n257), .b(new_n259), .c(new_n255), .d(new_n258), .o1(new_n260));
  160nm_ficinv00aa1n08x5       g165(.clk(new_n257), .clkout(new_n261));
  nanp02aa1n02x5               g166(.a(new_n255), .b(new_n258), .o1(new_n262));
  160nm_ficinv00aa1n08x5       g167(.clk(new_n259), .clkout(new_n263));
  aoi012aa1n02x5               g168(.a(new_n263), .b(new_n262), .c(new_n261), .o1(new_n264));
  norp02aa1n02x5               g169(.a(new_n264), .b(new_n260), .o1(\s[26] ));
  nanb02aa1n02x5               g170(.a(new_n166), .b(new_n167), .out0(new_n266));
  norp03aa1n02x5               g171(.a(new_n163), .b(new_n170), .c(new_n266), .o1(new_n267));
  norb03aa1n02x5               g172(.a(new_n131), .b(new_n132), .c(new_n145), .out0(new_n268));
  norp03aa1n02x5               g173(.a(new_n162), .b(new_n170), .c(new_n266), .o1(new_n269));
  oai012aa1n02x5               g174(.a(new_n269), .b(new_n268), .c(new_n150), .o1(new_n270));
  nona32aa1n02x4               g175(.a(new_n270), .b(new_n182), .c(new_n267), .d(new_n168), .out0(new_n271));
  and002aa1n02x5               g176(.a(new_n259), .b(new_n258), .o(new_n272));
  160nm_ficinv00aa1n08x5       g177(.clk(new_n272), .clkout(new_n273));
  norp03aa1n02x5               g178(.a(new_n273), .b(new_n250), .c(new_n211), .o1(new_n274));
  aoai13aa1n02x5               g179(.a(new_n274), .b(new_n271), .c(new_n138), .d(new_n178), .o1(new_n275));
  oao003aa1n02x5               g180(.a(\a[26] ), .b(\b[25] ), .c(new_n261), .carry(new_n276));
  aobi12aa1n02x5               g181(.a(new_n276), .b(new_n253), .c(new_n272), .out0(new_n277));
  xorc02aa1n02x5               g182(.a(\a[27] ), .b(\b[26] ), .out0(new_n278));
  xnbna2aa1n03x5               g183(.a(new_n278), .b(new_n275), .c(new_n277), .out0(\s[27] ));
  norp02aa1n02x5               g184(.a(\b[26] ), .b(\a[27] ), .o1(new_n280));
  160nm_ficinv00aa1n08x5       g185(.clk(new_n280), .clkout(new_n281));
  160nm_ficinv00aa1n08x5       g186(.clk(new_n278), .clkout(new_n282));
  aoi012aa1n02x5               g187(.a(new_n282), .b(new_n275), .c(new_n277), .o1(new_n283));
  xnrc02aa1n02x5               g188(.a(\b[27] ), .b(\a[28] ), .out0(new_n284));
  nano22aa1n02x4               g189(.a(new_n283), .b(new_n281), .c(new_n284), .out0(new_n285));
  nanp02aa1n02x5               g190(.a(new_n213), .b(new_n244), .o1(new_n286));
  aoai13aa1n02x5               g191(.a(new_n276), .b(new_n273), .c(new_n286), .d(new_n252), .o1(new_n287));
  aoai13aa1n02x5               g192(.a(new_n278), .b(new_n287), .c(new_n189), .d(new_n274), .o1(new_n288));
  aoi012aa1n02x5               g193(.a(new_n284), .b(new_n288), .c(new_n281), .o1(new_n289));
  norp02aa1n02x5               g194(.a(new_n289), .b(new_n285), .o1(\s[28] ));
  norb02aa1n02x5               g195(.a(new_n278), .b(new_n284), .out0(new_n291));
  aoai13aa1n02x5               g196(.a(new_n291), .b(new_n287), .c(new_n189), .d(new_n274), .o1(new_n292));
  oao003aa1n02x5               g197(.a(\a[28] ), .b(\b[27] ), .c(new_n281), .carry(new_n293));
  xnrc02aa1n02x5               g198(.a(\b[28] ), .b(\a[29] ), .out0(new_n294));
  aoi012aa1n02x5               g199(.a(new_n294), .b(new_n292), .c(new_n293), .o1(new_n295));
  160nm_ficinv00aa1n08x5       g200(.clk(new_n291), .clkout(new_n296));
  aoi012aa1n02x5               g201(.a(new_n296), .b(new_n275), .c(new_n277), .o1(new_n297));
  nano22aa1n02x4               g202(.a(new_n297), .b(new_n293), .c(new_n294), .out0(new_n298));
  norp02aa1n02x5               g203(.a(new_n295), .b(new_n298), .o1(\s[29] ));
  xorb03aa1n02x5               g204(.a(new_n108), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g205(.a(new_n278), .b(new_n294), .c(new_n284), .out0(new_n301));
  aoai13aa1n02x5               g206(.a(new_n301), .b(new_n287), .c(new_n189), .d(new_n274), .o1(new_n302));
  oao003aa1n02x5               g207(.a(\a[29] ), .b(\b[28] ), .c(new_n293), .carry(new_n303));
  xnrc02aa1n02x5               g208(.a(\b[29] ), .b(\a[30] ), .out0(new_n304));
  aoi012aa1n02x5               g209(.a(new_n304), .b(new_n302), .c(new_n303), .o1(new_n305));
  160nm_ficinv00aa1n08x5       g210(.clk(new_n301), .clkout(new_n306));
  aoi012aa1n02x5               g211(.a(new_n306), .b(new_n275), .c(new_n277), .o1(new_n307));
  nano22aa1n02x4               g212(.a(new_n307), .b(new_n303), .c(new_n304), .out0(new_n308));
  norp02aa1n02x5               g213(.a(new_n305), .b(new_n308), .o1(\s[30] ));
  xnrc02aa1n02x5               g214(.a(\b[30] ), .b(\a[31] ), .out0(new_n310));
  norb02aa1n02x5               g215(.a(new_n301), .b(new_n304), .out0(new_n311));
  160nm_ficinv00aa1n08x5       g216(.clk(new_n311), .clkout(new_n312));
  aoi012aa1n02x5               g217(.a(new_n312), .b(new_n275), .c(new_n277), .o1(new_n313));
  oao003aa1n02x5               g218(.a(\a[30] ), .b(\b[29] ), .c(new_n303), .carry(new_n314));
  nano22aa1n02x4               g219(.a(new_n313), .b(new_n310), .c(new_n314), .out0(new_n315));
  aoai13aa1n02x5               g220(.a(new_n311), .b(new_n287), .c(new_n189), .d(new_n274), .o1(new_n316));
  aoi012aa1n02x5               g221(.a(new_n310), .b(new_n316), .c(new_n314), .o1(new_n317));
  norp02aa1n02x5               g222(.a(new_n317), .b(new_n315), .o1(\s[31] ));
  xnrb03aa1n02x5               g223(.a(new_n110), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g224(.a(\a[3] ), .b(\b[2] ), .c(new_n110), .o1(new_n320));
  xorb03aa1n02x5               g225(.a(new_n320), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g226(.a(new_n117), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  160nm_ficinv00aa1n08x5       g227(.clk(new_n117), .clkout(new_n323));
  oaoi03aa1n02x5               g228(.a(\a[5] ), .b(\b[4] ), .c(new_n323), .o1(new_n324));
  xorb03aa1n02x5               g229(.a(new_n324), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  oai013aa1n02x4               g230(.a(new_n119), .b(new_n323), .c(new_n104), .d(new_n105), .o1(new_n326));
  xorb03aa1n02x5               g231(.a(new_n326), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoi012aa1n02x5               g232(.a(new_n101), .b(new_n326), .c(new_n102), .o1(new_n328));
  xnrb03aa1n02x5               g233(.a(new_n328), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xnbna2aa1n03x5               g234(.a(new_n123), .b(new_n136), .c(new_n137), .out0(\s[9] ));
endmodule


