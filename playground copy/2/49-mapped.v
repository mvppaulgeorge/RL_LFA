// Benchmark "adder" written by ABC on Wed Jul 10 16:50:05 2024

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
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n138, new_n139,
    new_n141, new_n142, new_n143, new_n144, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n154, new_n155, new_n156,
    new_n157, new_n159, new_n160, new_n161, new_n162, new_n163, new_n164,
    new_n165, new_n166, new_n168, new_n169, new_n170, new_n171, new_n173,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n183, new_n184, new_n185, new_n186, new_n188, new_n189,
    new_n190, new_n191, new_n192, new_n193, new_n194, new_n195, new_n196,
    new_n197, new_n198, new_n199, new_n200, new_n203, new_n204, new_n205,
    new_n206, new_n207, new_n208, new_n209, new_n210, new_n211, new_n213,
    new_n214, new_n215, new_n216, new_n217, new_n218, new_n219, new_n220,
    new_n221, new_n222, new_n223, new_n225, new_n226, new_n227, new_n228,
    new_n229, new_n230, new_n232, new_n233, new_n234, new_n235, new_n236,
    new_n237, new_n238, new_n239, new_n241, new_n242, new_n243, new_n244,
    new_n245, new_n246, new_n247, new_n248, new_n250, new_n251, new_n252,
    new_n253, new_n254, new_n255, new_n256, new_n257, new_n258, new_n259,
    new_n260, new_n261, new_n263, new_n264, new_n265, new_n266, new_n267,
    new_n268, new_n270, new_n271, new_n272, new_n273, new_n274, new_n275,
    new_n277, new_n278, new_n279, new_n280, new_n281, new_n282, new_n283,
    new_n284, new_n285, new_n286, new_n288, new_n289, new_n290, new_n291,
    new_n292, new_n293, new_n294, new_n297, new_n298, new_n299, new_n300,
    new_n301, new_n302, new_n303, new_n305, new_n306, new_n307, new_n308,
    new_n309, new_n310, new_n311, new_n314, new_n317, new_n319, new_n321;
  160nm_ficinv00aa1n08x5       g000(.clk(\a[0] ), .clkout(\s[0] ));
  norp02aa1n02x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  160nm_ficinv00aa1n08x5       g002(.clk(new_n97), .clkout(new_n98));
  norp02aa1n02x5               g003(.a(\b[1] ), .b(\a[2] ), .o1(new_n99));
  nanp02aa1n02x5               g004(.a(\b[0] ), .b(\a[1] ), .o1(new_n100));
  nanp02aa1n02x5               g005(.a(\b[1] ), .b(\a[2] ), .o1(new_n101));
  aoi012aa1n02x5               g006(.a(new_n99), .b(new_n100), .c(new_n101), .o1(new_n102));
  norp02aa1n02x5               g007(.a(\b[3] ), .b(\a[4] ), .o1(new_n103));
  nanp02aa1n02x5               g008(.a(\b[3] ), .b(\a[4] ), .o1(new_n104));
  norp02aa1n02x5               g009(.a(\b[2] ), .b(\a[3] ), .o1(new_n105));
  nanp02aa1n02x5               g010(.a(\b[2] ), .b(\a[3] ), .o1(new_n106));
  nona23aa1n02x4               g011(.a(new_n106), .b(new_n104), .c(new_n103), .d(new_n105), .out0(new_n107));
  aoi012aa1n02x5               g012(.a(new_n103), .b(new_n105), .c(new_n104), .o1(new_n108));
  oai012aa1n02x5               g013(.a(new_n108), .b(new_n107), .c(new_n102), .o1(new_n109));
  norp02aa1n02x5               g014(.a(\b[7] ), .b(\a[8] ), .o1(new_n110));
  nanp02aa1n02x5               g015(.a(\b[7] ), .b(\a[8] ), .o1(new_n111));
  norp02aa1n02x5               g016(.a(\b[6] ), .b(\a[7] ), .o1(new_n112));
  nanp02aa1n02x5               g017(.a(\b[6] ), .b(\a[7] ), .o1(new_n113));
  nona23aa1n02x4               g018(.a(new_n113), .b(new_n111), .c(new_n110), .d(new_n112), .out0(new_n114));
  xnrc02aa1n02x5               g019(.a(\b[5] ), .b(\a[6] ), .out0(new_n115));
  xnrc02aa1n02x5               g020(.a(\b[4] ), .b(\a[5] ), .out0(new_n116));
  norp03aa1n02x5               g021(.a(new_n114), .b(new_n115), .c(new_n116), .o1(new_n117));
  nanp02aa1n02x5               g022(.a(new_n109), .b(new_n117), .o1(new_n118));
  160nm_ficinv00aa1n08x5       g023(.clk(\a[5] ), .clkout(new_n119));
  160nm_ficinv00aa1n08x5       g024(.clk(\b[4] ), .clkout(new_n120));
  nanp02aa1n02x5               g025(.a(new_n120), .b(new_n119), .o1(new_n121));
  oaoi03aa1n02x5               g026(.a(\a[6] ), .b(\b[5] ), .c(new_n121), .o1(new_n122));
  160nm_fiao0012aa1n02p5x5     g027(.a(new_n110), .b(new_n112), .c(new_n111), .o(new_n123));
  aoib12aa1n02x5               g028(.a(new_n123), .b(new_n122), .c(new_n114), .out0(new_n124));
  nanp02aa1n02x5               g029(.a(\b[8] ), .b(\a[9] ), .o1(new_n125));
  norb02aa1n02x5               g030(.a(new_n125), .b(new_n97), .out0(new_n126));
  160nm_ficinv00aa1n08x5       g031(.clk(new_n126), .clkout(new_n127));
  aoai13aa1n02x5               g032(.a(new_n98), .b(new_n127), .c(new_n118), .d(new_n124), .o1(new_n128));
  xorb03aa1n02x5               g033(.a(new_n128), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  norp02aa1n02x5               g034(.a(\b[9] ), .b(\a[10] ), .o1(new_n130));
  nanp02aa1n02x5               g035(.a(\b[9] ), .b(\a[10] ), .o1(new_n131));
  norb02aa1n02x5               g036(.a(new_n131), .b(new_n130), .out0(new_n132));
  norp02aa1n02x5               g037(.a(\b[10] ), .b(\a[11] ), .o1(new_n133));
  nanp02aa1n02x5               g038(.a(\b[10] ), .b(\a[11] ), .o1(new_n134));
  norb02aa1n02x5               g039(.a(new_n134), .b(new_n133), .out0(new_n135));
  oai012aa1n02x5               g040(.a(new_n131), .b(new_n130), .c(new_n97), .o1(new_n136));
  160nm_ficinv00aa1n08x5       g041(.clk(new_n136), .clkout(new_n137));
  aoai13aa1n02x5               g042(.a(new_n135), .b(new_n137), .c(new_n128), .d(new_n132), .o1(new_n138));
  aoi112aa1n02x5               g043(.a(new_n137), .b(new_n135), .c(new_n128), .d(new_n132), .o1(new_n139));
  norb02aa1n02x5               g044(.a(new_n138), .b(new_n139), .out0(\s[11] ));
  160nm_ficinv00aa1n08x5       g045(.clk(new_n133), .clkout(new_n141));
  norp02aa1n02x5               g046(.a(\b[11] ), .b(\a[12] ), .o1(new_n142));
  nanp02aa1n02x5               g047(.a(\b[11] ), .b(\a[12] ), .o1(new_n143));
  norb02aa1n02x5               g048(.a(new_n143), .b(new_n142), .out0(new_n144));
  xnbna2aa1n03x5               g049(.a(new_n144), .b(new_n138), .c(new_n141), .out0(\s[12] ));
  nona23aa1n02x4               g050(.a(new_n143), .b(new_n134), .c(new_n133), .d(new_n142), .out0(new_n146));
  nano22aa1n02x4               g051(.a(new_n146), .b(new_n126), .c(new_n132), .out0(new_n147));
  160nm_ficinv00aa1n08x5       g052(.clk(new_n147), .clkout(new_n148));
  oaoi03aa1n02x5               g053(.a(\a[12] ), .b(\b[11] ), .c(new_n141), .o1(new_n149));
  oabi12aa1n02x5               g054(.a(new_n149), .b(new_n146), .c(new_n136), .out0(new_n150));
  160nm_ficinv00aa1n08x5       g055(.clk(new_n150), .clkout(new_n151));
  aoai13aa1n02x5               g056(.a(new_n151), .b(new_n148), .c(new_n118), .d(new_n124), .o1(new_n152));
  xorb03aa1n02x5               g057(.a(new_n152), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  orn002aa1n02x5               g058(.a(\a[13] ), .b(\b[12] ), .o(new_n154));
  xnrc02aa1n02x5               g059(.a(\b[12] ), .b(\a[13] ), .out0(new_n155));
  nanb02aa1n02x5               g060(.a(new_n155), .b(new_n152), .out0(new_n156));
  xnrc02aa1n02x5               g061(.a(\b[13] ), .b(\a[14] ), .out0(new_n157));
  xobna2aa1n03x5               g062(.a(new_n157), .b(new_n156), .c(new_n154), .out0(\s[14] ));
  norp02aa1n02x5               g063(.a(\b[14] ), .b(\a[15] ), .o1(new_n159));
  nanp02aa1n02x5               g064(.a(\b[14] ), .b(\a[15] ), .o1(new_n160));
  nanb02aa1n02x5               g065(.a(new_n159), .b(new_n160), .out0(new_n161));
  160nm_ficinv00aa1n08x5       g066(.clk(new_n161), .clkout(new_n162));
  norp02aa1n02x5               g067(.a(new_n157), .b(new_n155), .o1(new_n163));
  oaoi03aa1n02x5               g068(.a(\a[14] ), .b(\b[13] ), .c(new_n154), .o1(new_n164));
  aoai13aa1n02x5               g069(.a(new_n162), .b(new_n164), .c(new_n152), .d(new_n163), .o1(new_n165));
  aoi112aa1n02x5               g070(.a(new_n162), .b(new_n164), .c(new_n152), .d(new_n163), .o1(new_n166));
  norb02aa1n02x5               g071(.a(new_n165), .b(new_n166), .out0(\s[15] ));
  160nm_ficinv00aa1n08x5       g072(.clk(new_n159), .clkout(new_n168));
  norp02aa1n02x5               g073(.a(\b[15] ), .b(\a[16] ), .o1(new_n169));
  nanp02aa1n02x5               g074(.a(\b[15] ), .b(\a[16] ), .o1(new_n170));
  nanb02aa1n02x5               g075(.a(new_n169), .b(new_n170), .out0(new_n171));
  xobna2aa1n03x5               g076(.a(new_n171), .b(new_n165), .c(new_n168), .out0(\s[16] ));
  aoi012aa1n02x5               g077(.a(new_n110), .b(new_n112), .c(new_n111), .o1(new_n173));
  oaib12aa1n02x5               g078(.a(new_n173), .b(new_n114), .c(new_n122), .out0(new_n174));
  aoi012aa1n02x5               g079(.a(new_n174), .b(new_n109), .c(new_n117), .o1(new_n175));
  nona23aa1n02x4               g080(.a(new_n170), .b(new_n160), .c(new_n159), .d(new_n169), .out0(new_n176));
  nona32aa1n02x4               g081(.a(new_n147), .b(new_n176), .c(new_n157), .d(new_n155), .out0(new_n177));
  160nm_ficinv00aa1n08x5       g082(.clk(new_n176), .clkout(new_n178));
  aoai13aa1n02x5               g083(.a(new_n178), .b(new_n164), .c(new_n150), .d(new_n163), .o1(new_n179));
  aoi012aa1n02x5               g084(.a(new_n169), .b(new_n159), .c(new_n170), .o1(new_n180));
  oai112aa1n02x5               g085(.a(new_n179), .b(new_n180), .c(new_n175), .d(new_n177), .o1(new_n181));
  xorb03aa1n02x5               g086(.a(new_n181), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  160nm_ficinv00aa1n08x5       g087(.clk(\a[18] ), .clkout(new_n183));
  160nm_ficinv00aa1n08x5       g088(.clk(\a[17] ), .clkout(new_n184));
  160nm_ficinv00aa1n08x5       g089(.clk(\b[16] ), .clkout(new_n185));
  oaoi03aa1n02x5               g090(.a(new_n184), .b(new_n185), .c(new_n181), .o1(new_n186));
  xorb03aa1n02x5               g091(.a(new_n186), .b(\b[17] ), .c(new_n183), .out0(\s[18] ));
  aoi012aa1n02x5               g092(.a(new_n177), .b(new_n118), .c(new_n124), .o1(new_n188));
  160nm_ficinv00aa1n08x5       g093(.clk(new_n164), .clkout(new_n189));
  nano23aa1n02x4               g094(.a(new_n133), .b(new_n142), .c(new_n143), .d(new_n134), .out0(new_n190));
  aoai13aa1n02x5               g095(.a(new_n163), .b(new_n149), .c(new_n190), .d(new_n137), .o1(new_n191));
  aoai13aa1n02x5               g096(.a(new_n180), .b(new_n176), .c(new_n191), .d(new_n189), .o1(new_n192));
  xroi22aa1d04x5               g097(.a(new_n184), .b(\b[16] ), .c(new_n183), .d(\b[17] ), .out0(new_n193));
  oai012aa1n02x5               g098(.a(new_n193), .b(new_n192), .c(new_n188), .o1(new_n194));
  oai022aa1n02x5               g099(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n195));
  oaib12aa1n02x5               g100(.a(new_n195), .b(new_n183), .c(\b[17] ), .out0(new_n196));
  norp02aa1n02x5               g101(.a(\b[18] ), .b(\a[19] ), .o1(new_n197));
  nanp02aa1n02x5               g102(.a(\b[18] ), .b(\a[19] ), .o1(new_n198));
  nanb02aa1n02x5               g103(.a(new_n197), .b(new_n198), .out0(new_n199));
  160nm_ficinv00aa1n08x5       g104(.clk(new_n199), .clkout(new_n200));
  xnbna2aa1n03x5               g105(.a(new_n200), .b(new_n194), .c(new_n196), .out0(\s[19] ));
  xnrc02aa1n02x5               g106(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  160nm_ficinv00aa1n08x5       g107(.clk(new_n197), .clkout(new_n203));
  aoi012aa1n02x5               g108(.a(new_n199), .b(new_n194), .c(new_n196), .o1(new_n204));
  norp02aa1n02x5               g109(.a(\b[19] ), .b(\a[20] ), .o1(new_n205));
  nanp02aa1n02x5               g110(.a(\b[19] ), .b(\a[20] ), .o1(new_n206));
  nanb02aa1n02x5               g111(.a(new_n205), .b(new_n206), .out0(new_n207));
  nano22aa1n02x4               g112(.a(new_n204), .b(new_n203), .c(new_n207), .out0(new_n208));
  160nm_ficinv00aa1n08x5       g113(.clk(new_n196), .clkout(new_n209));
  aoai13aa1n02x5               g114(.a(new_n200), .b(new_n209), .c(new_n181), .d(new_n193), .o1(new_n210));
  aoi012aa1n02x5               g115(.a(new_n207), .b(new_n210), .c(new_n203), .o1(new_n211));
  norp02aa1n02x5               g116(.a(new_n211), .b(new_n208), .o1(\s[20] ));
  nano23aa1n02x4               g117(.a(new_n197), .b(new_n205), .c(new_n206), .d(new_n198), .out0(new_n213));
  nanp02aa1n02x5               g118(.a(new_n193), .b(new_n213), .o1(new_n214));
  160nm_ficinv00aa1n08x5       g119(.clk(new_n214), .clkout(new_n215));
  oai012aa1n02x5               g120(.a(new_n215), .b(new_n192), .c(new_n188), .o1(new_n216));
  nona23aa1n02x4               g121(.a(new_n206), .b(new_n198), .c(new_n197), .d(new_n205), .out0(new_n217));
  aoi012aa1n02x5               g122(.a(new_n205), .b(new_n197), .c(new_n206), .o1(new_n218));
  oai012aa1n02x5               g123(.a(new_n218), .b(new_n217), .c(new_n196), .o1(new_n219));
  160nm_ficinv00aa1n08x5       g124(.clk(new_n219), .clkout(new_n220));
  norp02aa1n02x5               g125(.a(\b[20] ), .b(\a[21] ), .o1(new_n221));
  nanp02aa1n02x5               g126(.a(\b[20] ), .b(\a[21] ), .o1(new_n222));
  norb02aa1n02x5               g127(.a(new_n222), .b(new_n221), .out0(new_n223));
  xnbna2aa1n03x5               g128(.a(new_n223), .b(new_n216), .c(new_n220), .out0(\s[21] ));
  160nm_ficinv00aa1n08x5       g129(.clk(new_n221), .clkout(new_n225));
  aobi12aa1n02x5               g130(.a(new_n223), .b(new_n216), .c(new_n220), .out0(new_n226));
  xnrc02aa1n02x5               g131(.a(\b[21] ), .b(\a[22] ), .out0(new_n227));
  nano22aa1n02x4               g132(.a(new_n226), .b(new_n225), .c(new_n227), .out0(new_n228));
  aoai13aa1n02x5               g133(.a(new_n223), .b(new_n219), .c(new_n181), .d(new_n215), .o1(new_n229));
  aoi012aa1n02x5               g134(.a(new_n227), .b(new_n229), .c(new_n225), .o1(new_n230));
  norp02aa1n02x5               g135(.a(new_n230), .b(new_n228), .o1(\s[22] ));
  nano22aa1n02x4               g136(.a(new_n227), .b(new_n225), .c(new_n222), .out0(new_n232));
  and003aa1n02x5               g137(.a(new_n193), .b(new_n232), .c(new_n213), .o(new_n233));
  oai012aa1n02x5               g138(.a(new_n233), .b(new_n192), .c(new_n188), .o1(new_n234));
  oao003aa1n02x5               g139(.a(\a[22] ), .b(\b[21] ), .c(new_n225), .carry(new_n235));
  160nm_ficinv00aa1n08x5       g140(.clk(new_n235), .clkout(new_n236));
  aoi012aa1n02x5               g141(.a(new_n236), .b(new_n219), .c(new_n232), .o1(new_n237));
  xnrc02aa1n02x5               g142(.a(\b[22] ), .b(\a[23] ), .out0(new_n238));
  160nm_ficinv00aa1n08x5       g143(.clk(new_n238), .clkout(new_n239));
  xnbna2aa1n03x5               g144(.a(new_n239), .b(new_n234), .c(new_n237), .out0(\s[23] ));
  norp02aa1n02x5               g145(.a(\b[22] ), .b(\a[23] ), .o1(new_n241));
  160nm_ficinv00aa1n08x5       g146(.clk(new_n241), .clkout(new_n242));
  aoi012aa1n02x5               g147(.a(new_n238), .b(new_n234), .c(new_n237), .o1(new_n243));
  xnrc02aa1n02x5               g148(.a(\b[23] ), .b(\a[24] ), .out0(new_n244));
  nano22aa1n02x4               g149(.a(new_n243), .b(new_n242), .c(new_n244), .out0(new_n245));
  160nm_ficinv00aa1n08x5       g150(.clk(new_n237), .clkout(new_n246));
  aoai13aa1n02x5               g151(.a(new_n239), .b(new_n246), .c(new_n181), .d(new_n233), .o1(new_n247));
  aoi012aa1n02x5               g152(.a(new_n244), .b(new_n247), .c(new_n242), .o1(new_n248));
  norp02aa1n02x5               g153(.a(new_n248), .b(new_n245), .o1(\s[24] ));
  norp02aa1n02x5               g154(.a(new_n244), .b(new_n238), .o1(new_n250));
  nano22aa1n02x4               g155(.a(new_n214), .b(new_n232), .c(new_n250), .out0(new_n251));
  oai012aa1n02x5               g156(.a(new_n251), .b(new_n192), .c(new_n188), .o1(new_n252));
  160nm_ficinv00aa1n08x5       g157(.clk(new_n218), .clkout(new_n253));
  aoai13aa1n02x5               g158(.a(new_n232), .b(new_n253), .c(new_n213), .d(new_n209), .o1(new_n254));
  160nm_ficinv00aa1n08x5       g159(.clk(new_n250), .clkout(new_n255));
  oao003aa1n02x5               g160(.a(\a[24] ), .b(\b[23] ), .c(new_n242), .carry(new_n256));
  aoai13aa1n02x5               g161(.a(new_n256), .b(new_n255), .c(new_n254), .d(new_n235), .o1(new_n257));
  xnrc02aa1n02x5               g162(.a(\b[24] ), .b(\a[25] ), .out0(new_n258));
  aoib12aa1n02x5               g163(.a(new_n258), .b(new_n252), .c(new_n257), .out0(new_n259));
  160nm_ficinv00aa1n08x5       g164(.clk(new_n258), .clkout(new_n260));
  aoi112aa1n02x5               g165(.a(new_n260), .b(new_n257), .c(new_n181), .d(new_n251), .o1(new_n261));
  norp02aa1n02x5               g166(.a(new_n259), .b(new_n261), .o1(\s[25] ));
  norp02aa1n02x5               g167(.a(\b[24] ), .b(\a[25] ), .o1(new_n263));
  160nm_ficinv00aa1n08x5       g168(.clk(new_n263), .clkout(new_n264));
  xnrc02aa1n02x5               g169(.a(\b[25] ), .b(\a[26] ), .out0(new_n265));
  nano22aa1n02x4               g170(.a(new_n259), .b(new_n264), .c(new_n265), .out0(new_n266));
  aoai13aa1n02x5               g171(.a(new_n260), .b(new_n257), .c(new_n181), .d(new_n251), .o1(new_n267));
  aoi012aa1n02x5               g172(.a(new_n265), .b(new_n267), .c(new_n264), .o1(new_n268));
  norp02aa1n02x5               g173(.a(new_n268), .b(new_n266), .o1(\s[26] ));
  norp02aa1n02x5               g174(.a(new_n265), .b(new_n258), .o1(new_n270));
  nano32aa1n02x4               g175(.a(new_n214), .b(new_n270), .c(new_n232), .d(new_n250), .out0(new_n271));
  oai012aa1n02x5               g176(.a(new_n271), .b(new_n192), .c(new_n188), .o1(new_n272));
  oao003aa1n02x5               g177(.a(\a[26] ), .b(\b[25] ), .c(new_n264), .carry(new_n273));
  aobi12aa1n02x5               g178(.a(new_n273), .b(new_n257), .c(new_n270), .out0(new_n274));
  xorc02aa1n02x5               g179(.a(\a[27] ), .b(\b[26] ), .out0(new_n275));
  xnbna2aa1n03x5               g180(.a(new_n275), .b(new_n274), .c(new_n272), .out0(\s[27] ));
  norp02aa1n02x5               g181(.a(\b[26] ), .b(\a[27] ), .o1(new_n277));
  160nm_ficinv00aa1n08x5       g182(.clk(new_n277), .clkout(new_n278));
  aobi12aa1n02x5               g183(.a(new_n275), .b(new_n274), .c(new_n272), .out0(new_n279));
  xnrc02aa1n02x5               g184(.a(\b[27] ), .b(\a[28] ), .out0(new_n280));
  nano22aa1n02x4               g185(.a(new_n279), .b(new_n278), .c(new_n280), .out0(new_n281));
  aoai13aa1n02x5               g186(.a(new_n250), .b(new_n236), .c(new_n219), .d(new_n232), .o1(new_n282));
  160nm_ficinv00aa1n08x5       g187(.clk(new_n270), .clkout(new_n283));
  aoai13aa1n02x5               g188(.a(new_n273), .b(new_n283), .c(new_n282), .d(new_n256), .o1(new_n284));
  aoai13aa1n02x5               g189(.a(new_n275), .b(new_n284), .c(new_n181), .d(new_n271), .o1(new_n285));
  aoi012aa1n02x5               g190(.a(new_n280), .b(new_n285), .c(new_n278), .o1(new_n286));
  norp02aa1n02x5               g191(.a(new_n286), .b(new_n281), .o1(\s[28] ));
  norb02aa1n02x5               g192(.a(new_n275), .b(new_n280), .out0(new_n288));
  aoai13aa1n02x5               g193(.a(new_n288), .b(new_n284), .c(new_n181), .d(new_n271), .o1(new_n289));
  oao003aa1n02x5               g194(.a(\a[28] ), .b(\b[27] ), .c(new_n278), .carry(new_n290));
  xnrc02aa1n02x5               g195(.a(\b[28] ), .b(\a[29] ), .out0(new_n291));
  aoi012aa1n02x5               g196(.a(new_n291), .b(new_n289), .c(new_n290), .o1(new_n292));
  aobi12aa1n02x5               g197(.a(new_n288), .b(new_n274), .c(new_n272), .out0(new_n293));
  nano22aa1n02x4               g198(.a(new_n293), .b(new_n290), .c(new_n291), .out0(new_n294));
  norp02aa1n02x5               g199(.a(new_n292), .b(new_n294), .o1(\s[29] ));
  xorb03aa1n02x5               g200(.a(new_n100), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g201(.a(new_n275), .b(new_n291), .c(new_n280), .out0(new_n297));
  aoai13aa1n02x5               g202(.a(new_n297), .b(new_n284), .c(new_n181), .d(new_n271), .o1(new_n298));
  oao003aa1n02x5               g203(.a(\a[29] ), .b(\b[28] ), .c(new_n290), .carry(new_n299));
  xnrc02aa1n02x5               g204(.a(\b[29] ), .b(\a[30] ), .out0(new_n300));
  aoi012aa1n02x5               g205(.a(new_n300), .b(new_n298), .c(new_n299), .o1(new_n301));
  aobi12aa1n02x5               g206(.a(new_n297), .b(new_n274), .c(new_n272), .out0(new_n302));
  nano22aa1n02x4               g207(.a(new_n302), .b(new_n299), .c(new_n300), .out0(new_n303));
  norp02aa1n02x5               g208(.a(new_n301), .b(new_n303), .o1(\s[30] ));
  norb02aa1n02x5               g209(.a(new_n297), .b(new_n300), .out0(new_n305));
  aobi12aa1n02x5               g210(.a(new_n305), .b(new_n274), .c(new_n272), .out0(new_n306));
  oao003aa1n02x5               g211(.a(\a[30] ), .b(\b[29] ), .c(new_n299), .carry(new_n307));
  xnrc02aa1n02x5               g212(.a(\b[30] ), .b(\a[31] ), .out0(new_n308));
  nano22aa1n02x4               g213(.a(new_n306), .b(new_n307), .c(new_n308), .out0(new_n309));
  aoai13aa1n02x5               g214(.a(new_n305), .b(new_n284), .c(new_n181), .d(new_n271), .o1(new_n310));
  aoi012aa1n02x5               g215(.a(new_n308), .b(new_n310), .c(new_n307), .o1(new_n311));
  norp02aa1n02x5               g216(.a(new_n311), .b(new_n309), .o1(\s[31] ));
  xnrb03aa1n02x5               g217(.a(new_n102), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g218(.a(\a[3] ), .b(\b[2] ), .c(new_n102), .o1(new_n314));
  xorb03aa1n02x5               g219(.a(new_n314), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g220(.a(new_n109), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oaoi03aa1n02x5               g221(.a(new_n119), .b(new_n120), .c(new_n109), .o1(new_n317));
  xnrb03aa1n02x5               g222(.a(new_n317), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  oaoi03aa1n02x5               g223(.a(\a[6] ), .b(\b[5] ), .c(new_n317), .o1(new_n319));
  xorb03aa1n02x5               g224(.a(new_n319), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoi012aa1n02x5               g225(.a(new_n112), .b(new_n319), .c(new_n113), .o1(new_n321));
  xnrb03aa1n02x5               g226(.a(new_n321), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xnbna2aa1n03x5               g227(.a(new_n175), .b(new_n125), .c(new_n98), .out0(\s[9] ));
endmodule


