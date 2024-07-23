// Benchmark "adder" written by ABC on Thu Jul 11 12:57:26 2024

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
    new_n125, new_n126, new_n128, new_n129, new_n130, new_n131, new_n133,
    new_n134, new_n135, new_n136, new_n137, new_n138, new_n139, new_n141,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n149,
    new_n150, new_n151, new_n153, new_n154, new_n155, new_n156, new_n157,
    new_n159, new_n160, new_n161, new_n162, new_n163, new_n165, new_n166,
    new_n167, new_n168, new_n169, new_n170, new_n171, new_n172, new_n173,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n180, new_n181,
    new_n182, new_n183, new_n185, new_n186, new_n187, new_n188, new_n189,
    new_n190, new_n193, new_n194, new_n195, new_n196, new_n197, new_n198,
    new_n199, new_n201, new_n202, new_n203, new_n204, new_n205, new_n206,
    new_n207, new_n209, new_n210, new_n211, new_n212, new_n213, new_n215,
    new_n216, new_n217, new_n218, new_n219, new_n220, new_n221, new_n222,
    new_n224, new_n225, new_n226, new_n227, new_n228, new_n230, new_n231,
    new_n232, new_n233, new_n234, new_n235, new_n236, new_n237, new_n238,
    new_n239, new_n240, new_n241, new_n242, new_n243, new_n244, new_n245,
    new_n247, new_n248, new_n249, new_n250, new_n251, new_n253, new_n254,
    new_n255, new_n256, new_n257, new_n258, new_n259, new_n260, new_n262,
    new_n263, new_n264, new_n265, new_n266, new_n267, new_n268, new_n269,
    new_n270, new_n271, new_n272, new_n274, new_n275, new_n276, new_n277,
    new_n278, new_n279, new_n280, new_n283, new_n284, new_n285, new_n286,
    new_n287, new_n288, new_n289, new_n291, new_n292, new_n293, new_n294,
    new_n295, new_n296, new_n297, new_n300, new_n303, new_n304, new_n306,
    new_n307, new_n309, new_n311;
  160nm_ficinv00aa1n08x5       g000(.clk(\a[0] ), .clkout(\s[0] ));
  xorc02aa1n02x5               g001(.a(\a[10] ), .b(\b[9] ), .out0(new_n97));
  norp02aa1n02x5               g002(.a(\b[8] ), .b(\a[9] ), .o1(new_n98));
  160nm_ficinv00aa1n08x5       g003(.clk(new_n98), .clkout(new_n99));
  and002aa1n02x5               g004(.a(\b[3] ), .b(\a[4] ), .o(new_n100));
  160nm_ficinv00aa1n08x5       g005(.clk(\a[3] ), .clkout(new_n101));
  160nm_ficinv00aa1n08x5       g006(.clk(\b[2] ), .clkout(new_n102));
  nanp02aa1n02x5               g007(.a(new_n102), .b(new_n101), .o1(new_n103));
  nanp02aa1n02x5               g008(.a(\b[2] ), .b(\a[3] ), .o1(new_n104));
  nanp02aa1n02x5               g009(.a(new_n103), .b(new_n104), .o1(new_n105));
  nanp02aa1n02x5               g010(.a(\b[1] ), .b(\a[2] ), .o1(new_n106));
  norp02aa1n02x5               g011(.a(\b[1] ), .b(\a[2] ), .o1(new_n107));
  nanp02aa1n02x5               g012(.a(\b[0] ), .b(\a[1] ), .o1(new_n108));
  oai012aa1n02x5               g013(.a(new_n106), .b(new_n107), .c(new_n108), .o1(new_n109));
  oa0022aa1n02x5               g014(.a(\a[4] ), .b(\b[3] ), .c(\a[3] ), .d(\b[2] ), .o(new_n110));
  oaoi13aa1n02x5               g015(.a(new_n100), .b(new_n110), .c(new_n109), .d(new_n105), .o1(new_n111));
  norp02aa1n02x5               g016(.a(\b[7] ), .b(\a[8] ), .o1(new_n112));
  nanp02aa1n02x5               g017(.a(\b[7] ), .b(\a[8] ), .o1(new_n113));
  norp02aa1n02x5               g018(.a(\b[6] ), .b(\a[7] ), .o1(new_n114));
  nanp02aa1n02x5               g019(.a(\b[6] ), .b(\a[7] ), .o1(new_n115));
  nona23aa1n02x4               g020(.a(new_n115), .b(new_n113), .c(new_n112), .d(new_n114), .out0(new_n116));
  xnrc02aa1n02x5               g021(.a(\b[5] ), .b(\a[6] ), .out0(new_n117));
  xnrc02aa1n02x5               g022(.a(\b[4] ), .b(\a[5] ), .out0(new_n118));
  norp03aa1n02x5               g023(.a(new_n116), .b(new_n117), .c(new_n118), .o1(new_n119));
  160nm_ficinv00aa1n08x5       g024(.clk(new_n112), .clkout(new_n120));
  nanp02aa1n02x5               g025(.a(new_n114), .b(new_n113), .o1(new_n121));
  aoi112aa1n02x5               g026(.a(\b[4] ), .b(\a[5] ), .c(\a[6] ), .d(\b[5] ), .o1(new_n122));
  oab012aa1n02x4               g027(.a(new_n122), .b(\a[6] ), .c(\b[5] ), .out0(new_n123));
  oai112aa1n02x5               g028(.a(new_n120), .b(new_n121), .c(new_n116), .d(new_n123), .o1(new_n124));
  xorc02aa1n02x5               g029(.a(\a[9] ), .b(\b[8] ), .out0(new_n125));
  aoai13aa1n02x5               g030(.a(new_n125), .b(new_n124), .c(new_n111), .d(new_n119), .o1(new_n126));
  xnbna2aa1n03x5               g031(.a(new_n97), .b(new_n126), .c(new_n99), .out0(\s[10] ));
  160nm_ficinv00aa1n08x5       g032(.clk(new_n97), .clkout(new_n128));
  aoi112aa1n02x5               g033(.a(\b[8] ), .b(\a[9] ), .c(\a[10] ), .d(\b[9] ), .o1(new_n129));
  oab012aa1n02x4               g034(.a(new_n129), .b(\a[10] ), .c(\b[9] ), .out0(new_n130));
  aoai13aa1n02x5               g035(.a(new_n130), .b(new_n128), .c(new_n126), .d(new_n99), .o1(new_n131));
  xorb03aa1n02x5               g036(.a(new_n131), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  norp02aa1n02x5               g037(.a(\b[10] ), .b(\a[11] ), .o1(new_n133));
  nanp02aa1n02x5               g038(.a(\b[10] ), .b(\a[11] ), .o1(new_n134));
  norp02aa1n02x5               g039(.a(\b[11] ), .b(\a[12] ), .o1(new_n135));
  nanp02aa1n02x5               g040(.a(\b[11] ), .b(\a[12] ), .o1(new_n136));
  norb02aa1n02x5               g041(.a(new_n136), .b(new_n135), .out0(new_n137));
  aoi112aa1n02x5               g042(.a(new_n137), .b(new_n133), .c(new_n131), .d(new_n134), .o1(new_n138));
  aoai13aa1n02x5               g043(.a(new_n137), .b(new_n133), .c(new_n131), .d(new_n134), .o1(new_n139));
  norb02aa1n02x5               g044(.a(new_n139), .b(new_n138), .out0(\s[12] ));
  nona23aa1n02x4               g045(.a(new_n136), .b(new_n134), .c(new_n133), .d(new_n135), .out0(new_n141));
  nano22aa1n02x4               g046(.a(new_n141), .b(new_n97), .c(new_n125), .out0(new_n142));
  aoai13aa1n02x5               g047(.a(new_n142), .b(new_n124), .c(new_n111), .d(new_n119), .o1(new_n143));
  aoi012aa1n02x5               g048(.a(new_n135), .b(new_n133), .c(new_n136), .o1(new_n144));
  oai012aa1n02x5               g049(.a(new_n144), .b(new_n141), .c(new_n130), .o1(new_n145));
  160nm_ficinv00aa1n08x5       g050(.clk(new_n145), .clkout(new_n146));
  nanp02aa1n02x5               g051(.a(new_n143), .b(new_n146), .o1(new_n147));
  xorb03aa1n02x5               g052(.a(new_n147), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  norp02aa1n02x5               g053(.a(\b[12] ), .b(\a[13] ), .o1(new_n149));
  nanp02aa1n02x5               g054(.a(\b[12] ), .b(\a[13] ), .o1(new_n150));
  aoi012aa1n02x5               g055(.a(new_n149), .b(new_n147), .c(new_n150), .o1(new_n151));
  xnrb03aa1n02x5               g056(.a(new_n151), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  norp02aa1n02x5               g057(.a(\b[13] ), .b(\a[14] ), .o1(new_n153));
  nanp02aa1n02x5               g058(.a(\b[13] ), .b(\a[14] ), .o1(new_n154));
  nona23aa1n02x4               g059(.a(new_n154), .b(new_n150), .c(new_n149), .d(new_n153), .out0(new_n155));
  aoi012aa1n02x5               g060(.a(new_n153), .b(new_n149), .c(new_n154), .o1(new_n156));
  aoai13aa1n02x5               g061(.a(new_n156), .b(new_n155), .c(new_n143), .d(new_n146), .o1(new_n157));
  xorb03aa1n02x5               g062(.a(new_n157), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  norp02aa1n02x5               g063(.a(\b[14] ), .b(\a[15] ), .o1(new_n159));
  xorc02aa1n02x5               g064(.a(\a[15] ), .b(\b[14] ), .out0(new_n160));
  xorc02aa1n02x5               g065(.a(\a[16] ), .b(\b[15] ), .out0(new_n161));
  aoi112aa1n02x5               g066(.a(new_n161), .b(new_n159), .c(new_n157), .d(new_n160), .o1(new_n162));
  aoai13aa1n02x5               g067(.a(new_n161), .b(new_n159), .c(new_n157), .d(new_n160), .o1(new_n163));
  norb02aa1n02x5               g068(.a(new_n163), .b(new_n162), .out0(\s[16] ));
  nano23aa1n02x4               g069(.a(new_n133), .b(new_n135), .c(new_n136), .d(new_n134), .out0(new_n165));
  nano23aa1n02x4               g070(.a(new_n149), .b(new_n153), .c(new_n154), .d(new_n150), .out0(new_n166));
  nanp03aa1n02x5               g071(.a(new_n166), .b(new_n160), .c(new_n161), .o1(new_n167));
  nano32aa1n02x4               g072(.a(new_n167), .b(new_n165), .c(new_n125), .d(new_n97), .out0(new_n168));
  aoai13aa1n02x5               g073(.a(new_n168), .b(new_n124), .c(new_n111), .d(new_n119), .o1(new_n169));
  xnrc02aa1n02x5               g074(.a(\b[14] ), .b(\a[15] ), .out0(new_n170));
  xnrc02aa1n02x5               g075(.a(\b[15] ), .b(\a[16] ), .out0(new_n171));
  norp03aa1n02x5               g076(.a(new_n155), .b(new_n171), .c(new_n170), .o1(new_n172));
  aob012aa1n02x5               g077(.a(new_n159), .b(\b[15] ), .c(\a[16] ), .out0(new_n173));
  160nm_ficinv00aa1n08x5       g078(.clk(new_n173), .clkout(new_n174));
  nanb03aa1n02x5               g079(.a(new_n156), .b(new_n161), .c(new_n160), .out0(new_n175));
  oai012aa1n02x5               g080(.a(new_n175), .b(\b[15] ), .c(\a[16] ), .o1(new_n176));
  aoi112aa1n02x5               g081(.a(new_n176), .b(new_n174), .c(new_n145), .d(new_n172), .o1(new_n177));
  nanp02aa1n02x5               g082(.a(new_n169), .b(new_n177), .o1(new_n178));
  xorb03aa1n02x5               g083(.a(new_n178), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  160nm_ficinv00aa1n08x5       g084(.clk(\a[18] ), .clkout(new_n180));
  160nm_ficinv00aa1n08x5       g085(.clk(\a[17] ), .clkout(new_n181));
  160nm_ficinv00aa1n08x5       g086(.clk(\b[16] ), .clkout(new_n182));
  oaoi03aa1n02x5               g087(.a(new_n181), .b(new_n182), .c(new_n178), .o1(new_n183));
  xorb03aa1n02x5               g088(.a(new_n183), .b(\b[17] ), .c(new_n180), .out0(\s[18] ));
  xroi22aa1d04x5               g089(.a(new_n181), .b(\b[16] ), .c(new_n180), .d(\b[17] ), .out0(new_n185));
  160nm_ficinv00aa1n08x5       g090(.clk(new_n185), .clkout(new_n186));
  norp02aa1n02x5               g091(.a(\b[17] ), .b(\a[18] ), .o1(new_n187));
  nanp02aa1n02x5               g092(.a(\b[17] ), .b(\a[18] ), .o1(new_n188));
  aoi013aa1n02x4               g093(.a(new_n187), .b(new_n188), .c(new_n181), .d(new_n182), .o1(new_n189));
  aoai13aa1n02x5               g094(.a(new_n189), .b(new_n186), .c(new_n169), .d(new_n177), .o1(new_n190));
  xorb03aa1n02x5               g095(.a(new_n190), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g096(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  norp02aa1n02x5               g097(.a(\b[18] ), .b(\a[19] ), .o1(new_n193));
  nanp02aa1n02x5               g098(.a(\b[18] ), .b(\a[19] ), .o1(new_n194));
  norp02aa1n02x5               g099(.a(\b[19] ), .b(\a[20] ), .o1(new_n195));
  nanp02aa1n02x5               g100(.a(\b[19] ), .b(\a[20] ), .o1(new_n196));
  norb02aa1n02x5               g101(.a(new_n196), .b(new_n195), .out0(new_n197));
  aoi112aa1n02x5               g102(.a(new_n193), .b(new_n197), .c(new_n190), .d(new_n194), .o1(new_n198));
  aoai13aa1n02x5               g103(.a(new_n197), .b(new_n193), .c(new_n190), .d(new_n194), .o1(new_n199));
  norb02aa1n02x5               g104(.a(new_n199), .b(new_n198), .out0(\s[20] ));
  nano23aa1n02x4               g105(.a(new_n193), .b(new_n195), .c(new_n196), .d(new_n194), .out0(new_n201));
  nanp02aa1n02x5               g106(.a(new_n185), .b(new_n201), .o1(new_n202));
  nona23aa1n02x4               g107(.a(new_n196), .b(new_n194), .c(new_n193), .d(new_n195), .out0(new_n203));
  oai012aa1n02x5               g108(.a(new_n196), .b(new_n195), .c(new_n193), .o1(new_n204));
  oai012aa1n02x5               g109(.a(new_n204), .b(new_n203), .c(new_n189), .o1(new_n205));
  160nm_ficinv00aa1n08x5       g110(.clk(new_n205), .clkout(new_n206));
  aoai13aa1n02x5               g111(.a(new_n206), .b(new_n202), .c(new_n169), .d(new_n177), .o1(new_n207));
  xorb03aa1n02x5               g112(.a(new_n207), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g113(.a(\b[20] ), .b(\a[21] ), .o1(new_n209));
  xorc02aa1n02x5               g114(.a(\a[21] ), .b(\b[20] ), .out0(new_n210));
  xorc02aa1n02x5               g115(.a(\a[22] ), .b(\b[21] ), .out0(new_n211));
  aoi112aa1n02x5               g116(.a(new_n209), .b(new_n211), .c(new_n207), .d(new_n210), .o1(new_n212));
  aoai13aa1n02x5               g117(.a(new_n211), .b(new_n209), .c(new_n207), .d(new_n210), .o1(new_n213));
  norb02aa1n02x5               g118(.a(new_n213), .b(new_n212), .out0(\s[22] ));
  160nm_ficinv00aa1n08x5       g119(.clk(\a[21] ), .clkout(new_n215));
  160nm_ficinv00aa1n08x5       g120(.clk(\a[22] ), .clkout(new_n216));
  xroi22aa1d04x5               g121(.a(new_n215), .b(\b[20] ), .c(new_n216), .d(\b[21] ), .out0(new_n217));
  nanp03aa1n02x5               g122(.a(new_n217), .b(new_n185), .c(new_n201), .o1(new_n218));
  160nm_ficinv00aa1n08x5       g123(.clk(\b[21] ), .clkout(new_n219));
  oao003aa1n02x5               g124(.a(new_n216), .b(new_n219), .c(new_n209), .carry(new_n220));
  aoi012aa1n02x5               g125(.a(new_n220), .b(new_n205), .c(new_n217), .o1(new_n221));
  aoai13aa1n02x5               g126(.a(new_n221), .b(new_n218), .c(new_n169), .d(new_n177), .o1(new_n222));
  xorb03aa1n02x5               g127(.a(new_n222), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g128(.a(\b[22] ), .b(\a[23] ), .o1(new_n224));
  xorc02aa1n02x5               g129(.a(\a[23] ), .b(\b[22] ), .out0(new_n225));
  xorc02aa1n02x5               g130(.a(\a[24] ), .b(\b[23] ), .out0(new_n226));
  aoi112aa1n02x5               g131(.a(new_n224), .b(new_n226), .c(new_n222), .d(new_n225), .o1(new_n227));
  aoai13aa1n02x5               g132(.a(new_n226), .b(new_n224), .c(new_n222), .d(new_n225), .o1(new_n228));
  norb02aa1n02x5               g133(.a(new_n228), .b(new_n227), .out0(\s[24] ));
  160nm_ficinv00aa1n08x5       g134(.clk(\a[23] ), .clkout(new_n230));
  160nm_ficinv00aa1n08x5       g135(.clk(\a[24] ), .clkout(new_n231));
  xroi22aa1d04x5               g136(.a(new_n230), .b(\b[22] ), .c(new_n231), .d(\b[23] ), .out0(new_n232));
  nano22aa1n02x4               g137(.a(new_n202), .b(new_n217), .c(new_n232), .out0(new_n233));
  nona22aa1n02x4               g138(.a(new_n188), .b(\b[16] ), .c(\a[17] ), .out0(new_n234));
  oaib12aa1n02x5               g139(.a(new_n234), .b(\b[17] ), .c(new_n180), .out0(new_n235));
  160nm_ficinv00aa1n08x5       g140(.clk(new_n204), .clkout(new_n236));
  aoai13aa1n02x5               g141(.a(new_n217), .b(new_n236), .c(new_n201), .d(new_n235), .o1(new_n237));
  160nm_ficinv00aa1n08x5       g142(.clk(new_n220), .clkout(new_n238));
  160nm_ficinv00aa1n08x5       g143(.clk(new_n232), .clkout(new_n239));
  oai022aa1n02x5               g144(.a(\a[23] ), .b(\b[22] ), .c(\b[23] ), .d(\a[24] ), .o1(new_n240));
  oaib12aa1n02x5               g145(.a(new_n240), .b(new_n231), .c(\b[23] ), .out0(new_n241));
  aoai13aa1n02x5               g146(.a(new_n241), .b(new_n239), .c(new_n237), .d(new_n238), .o1(new_n242));
  xorc02aa1n02x5               g147(.a(\a[25] ), .b(\b[24] ), .out0(new_n243));
  aoai13aa1n02x5               g148(.a(new_n243), .b(new_n242), .c(new_n178), .d(new_n233), .o1(new_n244));
  aoi112aa1n02x5               g149(.a(new_n243), .b(new_n242), .c(new_n178), .d(new_n233), .o1(new_n245));
  norb02aa1n02x5               g150(.a(new_n244), .b(new_n245), .out0(\s[25] ));
  norp02aa1n02x5               g151(.a(\b[24] ), .b(\a[25] ), .o1(new_n247));
  xorc02aa1n02x5               g152(.a(\a[26] ), .b(\b[25] ), .out0(new_n248));
  nona22aa1n02x4               g153(.a(new_n244), .b(new_n248), .c(new_n247), .out0(new_n249));
  160nm_ficinv00aa1n08x5       g154(.clk(new_n247), .clkout(new_n250));
  aobi12aa1n02x5               g155(.a(new_n248), .b(new_n244), .c(new_n250), .out0(new_n251));
  norb02aa1n02x5               g156(.a(new_n249), .b(new_n251), .out0(\s[26] ));
  160nm_ficinv00aa1n08x5       g157(.clk(\a[25] ), .clkout(new_n253));
  160nm_ficinv00aa1n08x5       g158(.clk(\a[26] ), .clkout(new_n254));
  xroi22aa1d04x5               g159(.a(new_n253), .b(\b[24] ), .c(new_n254), .d(\b[25] ), .out0(new_n255));
  nano32aa1n02x4               g160(.a(new_n202), .b(new_n255), .c(new_n217), .d(new_n232), .out0(new_n256));
  nanp02aa1n02x5               g161(.a(new_n178), .b(new_n256), .o1(new_n257));
  oao003aa1n02x5               g162(.a(\a[26] ), .b(\b[25] ), .c(new_n250), .carry(new_n258));
  aobi12aa1n02x5               g163(.a(new_n258), .b(new_n242), .c(new_n255), .out0(new_n259));
  xorc02aa1n02x5               g164(.a(\a[27] ), .b(\b[26] ), .out0(new_n260));
  xnbna2aa1n03x5               g165(.a(new_n260), .b(new_n259), .c(new_n257), .out0(\s[27] ));
  norp02aa1n02x5               g166(.a(\b[26] ), .b(\a[27] ), .o1(new_n262));
  160nm_ficinv00aa1n08x5       g167(.clk(new_n262), .clkout(new_n263));
  aobi12aa1n02x5               g168(.a(new_n260), .b(new_n259), .c(new_n257), .out0(new_n264));
  xnrc02aa1n02x5               g169(.a(\b[27] ), .b(\a[28] ), .out0(new_n265));
  nano22aa1n02x4               g170(.a(new_n264), .b(new_n263), .c(new_n265), .out0(new_n266));
  aobi12aa1n02x5               g171(.a(new_n256), .b(new_n169), .c(new_n177), .out0(new_n267));
  aoai13aa1n02x5               g172(.a(new_n232), .b(new_n220), .c(new_n205), .d(new_n217), .o1(new_n268));
  160nm_ficinv00aa1n08x5       g173(.clk(new_n255), .clkout(new_n269));
  aoai13aa1n02x5               g174(.a(new_n258), .b(new_n269), .c(new_n268), .d(new_n241), .o1(new_n270));
  oai012aa1n02x5               g175(.a(new_n260), .b(new_n270), .c(new_n267), .o1(new_n271));
  aoi012aa1n02x5               g176(.a(new_n265), .b(new_n271), .c(new_n263), .o1(new_n272));
  norp02aa1n02x5               g177(.a(new_n272), .b(new_n266), .o1(\s[28] ));
  norb02aa1n02x5               g178(.a(new_n260), .b(new_n265), .out0(new_n274));
  aobi12aa1n02x5               g179(.a(new_n274), .b(new_n259), .c(new_n257), .out0(new_n275));
  oao003aa1n02x5               g180(.a(\a[28] ), .b(\b[27] ), .c(new_n263), .carry(new_n276));
  xnrc02aa1n02x5               g181(.a(\b[28] ), .b(\a[29] ), .out0(new_n277));
  nano22aa1n02x4               g182(.a(new_n275), .b(new_n276), .c(new_n277), .out0(new_n278));
  oai012aa1n02x5               g183(.a(new_n274), .b(new_n270), .c(new_n267), .o1(new_n279));
  aoi012aa1n02x5               g184(.a(new_n277), .b(new_n279), .c(new_n276), .o1(new_n280));
  norp02aa1n02x5               g185(.a(new_n280), .b(new_n278), .o1(\s[29] ));
  xorb03aa1n02x5               g186(.a(new_n108), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g187(.a(new_n260), .b(new_n277), .c(new_n265), .out0(new_n283));
  aobi12aa1n02x5               g188(.a(new_n283), .b(new_n259), .c(new_n257), .out0(new_n284));
  oao003aa1n02x5               g189(.a(\a[29] ), .b(\b[28] ), .c(new_n276), .carry(new_n285));
  xnrc02aa1n02x5               g190(.a(\b[29] ), .b(\a[30] ), .out0(new_n286));
  nano22aa1n02x4               g191(.a(new_n284), .b(new_n285), .c(new_n286), .out0(new_n287));
  oai012aa1n02x5               g192(.a(new_n283), .b(new_n270), .c(new_n267), .o1(new_n288));
  aoi012aa1n02x5               g193(.a(new_n286), .b(new_n288), .c(new_n285), .o1(new_n289));
  norp02aa1n02x5               g194(.a(new_n289), .b(new_n287), .o1(\s[30] ));
  norb02aa1n02x5               g195(.a(new_n283), .b(new_n286), .out0(new_n291));
  aobi12aa1n02x5               g196(.a(new_n291), .b(new_n259), .c(new_n257), .out0(new_n292));
  oao003aa1n02x5               g197(.a(\a[30] ), .b(\b[29] ), .c(new_n285), .carry(new_n293));
  xnrc02aa1n02x5               g198(.a(\b[30] ), .b(\a[31] ), .out0(new_n294));
  nano22aa1n02x4               g199(.a(new_n292), .b(new_n293), .c(new_n294), .out0(new_n295));
  oai012aa1n02x5               g200(.a(new_n291), .b(new_n270), .c(new_n267), .o1(new_n296));
  aoi012aa1n02x5               g201(.a(new_n294), .b(new_n296), .c(new_n293), .o1(new_n297));
  norp02aa1n02x5               g202(.a(new_n297), .b(new_n295), .o1(\s[31] ));
  xnbna2aa1n03x5               g203(.a(new_n109), .b(new_n103), .c(new_n104), .out0(\s[3] ));
  oaoi03aa1n02x5               g204(.a(\a[3] ), .b(\b[2] ), .c(new_n109), .o1(new_n300));
  xorb03aa1n02x5               g205(.a(new_n300), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g206(.a(new_n111), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  orn002aa1n02x5               g207(.a(\a[5] ), .b(\b[4] ), .o(new_n303));
  nanb02aa1n02x5               g208(.a(new_n118), .b(new_n111), .out0(new_n304));
  xobna2aa1n03x5               g209(.a(new_n117), .b(new_n304), .c(new_n303), .out0(\s[6] ));
  norp02aa1n02x5               g210(.a(new_n118), .b(new_n117), .o1(new_n306));
  aobi12aa1n02x5               g211(.a(new_n123), .b(new_n111), .c(new_n306), .out0(new_n307));
  xnrb03aa1n02x5               g212(.a(new_n307), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  oaoi03aa1n02x5               g213(.a(\a[7] ), .b(\b[6] ), .c(new_n307), .o1(new_n309));
  xorb03aa1n02x5               g214(.a(new_n309), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  aoi112aa1n02x5               g215(.a(new_n124), .b(new_n125), .c(new_n111), .d(new_n119), .o1(new_n311));
  norb02aa1n02x5               g216(.a(new_n126), .b(new_n311), .out0(\s[9] ));
endmodule


